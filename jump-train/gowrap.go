package main

import (
	"fmt"
	"os"
	"unsafe"
)

// #cgo CXXFLAGS: -include gowrap.h -std=c++11
// #include <stdlib.h>
// #include "gowrap.h"
import "C"

func main() {
	argc := C.int(len(os.Args))
	argv := make([]*C.char, len(os.Args))
	for i,arg := range os.Args {
		argv[i] = C.CString(arg)
	}
	
	_exitCode,err := C.rd_main(argc, &argv[0])
	exitCode := int(_exitCode)
	
	for _,arg := range argv {
		C.free(unsafe.Pointer(arg))
	}
	
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
	if exitCode != 0 { os.Exit(exitCode) }
}
