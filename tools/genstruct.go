package main

import (
	"encoding/json"
	"encoding/xml"
	"flag"
	"fmt"
	"log"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
	"text/template"
	"unicode"
)

type Device struct {
	Name            string        `xml:"name"`
	Version         string        `xml:"version"`
	Description     string        `xml:"description"`
	AddressUnitBits number        `xml:"addressUnitBits"`
	Width           number        `xml:"width"`
	Size            number        `xml:"size"`       // peripheral register default
	ResetValue      number        `xml:"resetValue"` // peripheral register default
	ResetMask       number        `xml:"resetMask"`  // peripheral register default
	Peripherals     []*Peripheral `xml:"peripherals>peripheral"`
	Interrupts      []*Interrupt  // created from the peripherals
}

func (d *Device) PeripheralByName(s string) *Peripheral {
	for _, v := range d.Peripherals {
		if v.Name == s {
			return v
		}
	}
	return nil
}

type Peripheral struct {
	Name             string        `xml:"name"`
	DerivedFrom      string        `xml:"derivedFrom,attr"` // same type as this device
	Description      string        `xml:"description"`
	GroupName        string        `xml:"groupName"`
	BaseAddress      number        `xml:"baseAddress"`
	AddressBlockSize number        `xml:"addressBlock>Size"`
	Interrupts       []*Interrupt  `xml:"interrupt"`
	Registers        []*Register   `xml:"registers>register"`
	Extends          []*Peripheral `xml:-` // other peripheral types  that
}

func (p *Peripheral) RegisterByName(s string) *Register {
	for _, v := range p.Registers {
		if v.Name == s {
			return v
		}
	}
	return nil
}

func (p *Peripheral) FillRegisters() {
	var r []*Register
	var offset int64 // in units of bytes
	ridx := 0
	for _, v := range p.Registers {

		if len(v.Fields) > 0 {
			switch {
			case v.Fields[0].BitOffset+v.Fields[0].BitWidth <= 8:
				v.Size = 8
			case v.Fields[0].BitOffset+v.Fields[0].BitWidth <= 16:
				v.Size = 16
			}
		}

		// same address as previous: generate union field
		if len(r) > 1 && v.AddressOffset == r[len(r)-1].AddressOffset {
			r[len(r)-1].Union = append(r[len(r)-1].Union, v)
			continue
		}

		if v.AddressOffset%(v.Size/8) != 0 {
			log.Fatalln("Unaligned field %s:%s [%d] at %d bit", p.Name, v.Name, v.Size, v.AddressOffset)
		}

		gap := int64(v.AddressOffset) - offset

		if gap > 0 {
			r = append(r, &Register{Name: fmt.Sprintf("RESERVED%d", ridx), AddressOffset: number(offset), Size: 8, Access: "reserved", ArraySize: uint64(gap)})
			offset += gap
			ridx++
		}

		r = append(r, v)
		offset += int64(v.Size / 8)
	}
	p.Registers = r
}

type Interrupt struct {
	Name        string `xml:"name"`
	Description string `xml:"description"`
	Value       int    `xml:"value"`
}

type Register struct {
	Name          string   `xml:"name"`
	DisplayName   string   `xml:"displayName"`
	Description   string   `xml:"description"`
	AddressOffset number   `xml:"addressOffset"`
	Size          number   `xml:"size"`
	Access        string   `xml:"access"`
	ResetValue    number   `xml:"resetValue"`
	Fields        []*Field `xml:"fields>field"`
	// these are syntesized by analyzing the device
	Union     []*Register `xml:-`
	ArraySize uint64      `xml:-`
}

func (r *Register) FieldByName(s string) *Field {
	for _, v := range r.Fields {
		if v.Name == s {
			return v
		}
	}
	return nil
}

func (r *Register) HasEnumFields() bool {
	for _, v := range r.Fields {
		if v.BitWidth < r.Size {
			return true
		}
	}
	return false
}

type Field struct {
	Name        string `xml:"name"`
	Description string `xml:"description"`
	BitOffset   number `xml:"bitOffset"`
	BitWidth    number `xml:"bitWidth"`
}

func (f *Field) String() string {
	if f.BitWidth > 1 {
		return fmt.Sprintf("[%d:%d]%s", f.BitOffset+f.BitWidth-1, f.BitOffset, f.Name)
	}
	return fmt.Sprintf("[%d]%s", f.BitOffset, f.Name)
}

type byName []*Peripheral

func (b byName) Len() int           { return len(b) }
func (b byName) Swap(i, j int)      { b[i], b[j] = b[j], b[i] }
func (b byName) Less(i, j int) bool { return b[i].Name < b[j].Name }

type byValue []*Interrupt

func (b byValue) Len() int           { return len(b) }
func (b byValue) Swap(i, j int)      { b[i], b[j] = b[j], b[i] }
func (b byValue) Less(i, j int) bool { return b[i].Value < b[j].Value }

type byAddressOffset []*Register

func (b byAddressOffset) Len() int           { return len(b) }
func (b byAddressOffset) Swap(i, j int)      { b[i], b[j] = b[j], b[i] }
func (b byAddressOffset) Less(i, j int) bool { return b[i].AddressOffset < b[j].AddressOffset }

type byBitOffset []*Field

func (b byBitOffset) Len() int           { return len(b) }
func (b byBitOffset) Swap(i, j int)      { b[i], b[j] = b[j], b[i] }
func (b byBitOffset) Less(i, j int) bool { return b[i].BitOffset > b[j].BitOffset }

type number uint64

func (hn *number) UnmarshalXML(d *xml.Decoder, start xml.StartElement) error {
	var s string
	err := d.DecodeElement(&s, &start)
	if err != nil {
		return err
	}
	n, err := strconv.ParseUint(s, 0, 64)
	*hn = number(n)
	return err
}

func (hn number) Hex() string { return fmt.Sprintf("0x%X", uint64(hn)) }

var tmplfuncs = template.FuncMap{
	"lower": strings.ToLower,
	"upper": strings.ToUpper,
}

var fDebug = flag.Bool("d", false, "dump parsed device struct")

func main() {

	log.SetFlags(0)
	log.SetPrefix("svdgen: ")
	flag.Parse()

	if len(flag.Args()) != 2 {
		log.Fatalf("Usage: %s path/to/lang.tmpl path/to/dialect.xml", os.Args[0])
	}

	tmpl, err := template.New(filepath.Base(flag.Arg(0))).Funcs(tmplfuncs).ParseFiles(flag.Arg(0))
	if err != nil {
		log.Fatal(err)
	}
	log.Println("template file:", tmpl.Name())

	f, err := os.Open(flag.Arg(1))
	if err != nil {
		log.Fatal(err)
	}
	dname, fname := filepath.Split(f.Name())
	basename := strings.ToLower(strings.TrimSuffix(fname, filepath.Ext(fname)))
	log.Println(dname, fname, basename)
	var device Device

	if err := xml.NewDecoder(f).Decode(&device); err != nil {
		log.Fatal(err)
	}
	f.Close()

	index := map[string]string{}

	if device.AddressUnitBits != 8 || device.Width != 32 {
		log.Fatal("can only work on assuming AddressUnitBits = 8, got %d and device.Width = 32, got %d", device.AddressUnitBits, device.Width)
	}

	for _, v := range device.Peripherals {
		for _, w := range v.Registers {
			if w.Size != 32 {
				log.Println("%s %s register has bad size %d", v.Name, w.Name, w.Size)
			}
		}
	}

	// Cleanup whitespace in description fields, sort
	sort.Sort(byName(device.Peripherals))
	for _, v := range device.Peripherals {
		v.Description = strings.Join(strings.Fields(v.Description), " ")

		sort.Sort(byValue(v.Interrupts))
		for _, w := range v.Interrupts {
			w.Description = strings.Join(strings.Fields(w.Description), " ")
		}
		device.Interrupts = append(device.Interrupts, v.Interrupts...)

		sort.Sort(byAddressOffset(v.Registers))
		for _, w := range v.Registers {
			w.Description = strings.Join(strings.Fields(w.Description), " ")

			if len(w.Fields) == 0 {
				continue
			}

			sort.Sort(byBitOffset(w.Fields))

			if len(w.Fields) == int(w.Fields[0].BitOffset+w.Fields[0].BitWidth) && len(w.Fields) > 1 {
				// only 1 bit fields, dense from lsb upward
				t := nameTemplate(w.Fields[0].Name)
				allsame := true
				for _, v := range w.Fields[1:] {
					if t != nameTemplate(v.Name) {
						allsame = false
						break
					}
				}
				if allsame {
					w.Fields[0].BitWidth = w.Fields[0].BitOffset + 1
					w.Fields[0].BitOffset = 0
					w.Fields[0].Name = t
					w.Fields[0].Description = "Merged " + w.Fields[0].Description
					w.Fields = w.Fields[:1]
				}
			}

			for _, x := range w.Fields {
				x.Description = strings.Join(strings.Fields(x.Description), " ")

				index[v.Name+"::"+w.Name] += x.String()

			}
		}
	}
	sort.Sort(byValue(device.Interrupts))

	if *fDebug {
		enc := json.NewEncoder(os.Stdout)
		enc.SetIndent("", "\t")
		enc.Encode(&device)
		return
	}

	revindex := map[string][]string{}
	for k, v := range index {
		revindex[v] = append(revindex[v], k)
	}

	// for k, v := range revindex {
	// 	if len(v) > 1 {
	// 		fmt.Println(k, v)
	// 	}
	// }
	// log.Fatal()

	var timers []*Peripheral
	for _, v := range device.Peripherals {
		if v.DerivedFrom == "" && nameTemplate(v.Name) == nameTemplate("TIM1") {
			timers = append(timers, v)
			log.Println("timer: ", v.Name)
		}
	}

	var basicTimer = timers[0]
	for _, v := range timers[1:] {
		log.Println("adding", v.Name)
		basicTimer = intersectType(basicTimer, v)
	}
	basicTimer.Name = "BasicTimer"
	basicTimer.Description = "intersection of all timers"
	device.Peripherals = append(device.Peripherals, basicTimer)

	if false {
		var uarts []*Peripheral
		for _, v := range device.Peripherals {
			if v.DerivedFrom != "" {
				continue
			}
			if n := nameTemplate(v.Name); n == nameTemplate("USART1") || n == nameTemplate("UART1") {
				uarts = append(uarts, v)
				log.Println("uart: ", v.Name)
			}
		}
		var basicUart = uarts[0]
		for _, v := range uarts[1:] {
			log.Println("adding", v.Name)
			basicUart = intersectType(basicUart, v)
		}
		basicUart.Name = "BasicUart"
		basicUart.Description = "intersection of all uarts"
		device.Peripherals = append(device.Peripherals, basicUart)
	}

	if true {
		for k1, v1 := range device.Peripherals {
			for k2, v2 := range device.Peripherals {
				if k1 == k2 {
					continue
				}
				if v2.DerivedFrom != "" {
					continue
				}
				if isSuperset(v1, v2) {
					log.Printf("%s is super type of %s", v1.Name, v2.Name)
					v1.Extends = append(v1.Extends, v2)
				}
			}
		}
	}

	for _, v := range device.Peripherals {
		v.FillRegisters()
	}

	if err := tmpl.Execute(os.Stdout, &device); err != nil {
		log.Fatal(err)
	}
}

func intersectType(a, b *Peripheral) *Peripheral {
	p := &Peripheral{}
	for _, v := range a.Registers {
		vv := b.RegisterByName(v.Name)
		if vv == nil {
			continue
		}
		if vv.AddressOffset != v.AddressOffset {
			continue
		}
		if vv.Size != v.Size {
			continue
		}

		r := &Register{Name: v.Name, Description: v.Description, AddressOffset: v.AddressOffset, Size: v.Size}
		for _, fs := range v.Fields {
			fb := vv.FieldByName(fs.Name)
			if fb == nil {
				continue
			}
			if fb.BitOffset != fs.BitOffset {
				continue
			}
			if fb.BitWidth != fs.BitWidth {
				continue
			}
			r.Fields = append(r.Fields, fs)
		}

		if len(r.Fields) > 0 {
			p.Registers = append(p.Registers, r)
		}
	}
	if len(p.Registers) > 0 {
		return p
	}
	return nil
}

func isSuperset(big, small *Peripheral) bool {
	for _, v := range small.Registers {
		vv := big.RegisterByName(v.Name)
		if vv == nil {
			return false
		}
		if vv.AddressOffset != v.AddressOffset {
			return false
		}
		if vv.Size != v.Size {
			return false
		}
		for _, fs := range v.Fields {
			fb := vv.FieldByName(fs.Name)
			if fb == nil {
				return false
			}
			if fb.BitOffset != fs.BitOffset {
				return false
			}
			if fb.BitWidth != fs.BitWidth {
				return false
			}
		}
	}
	return true
}

func nameTemplate(s string) string {
	return strings.Join(strings.FieldsFunc(s+" ", unicode.IsDigit), "X")
}
