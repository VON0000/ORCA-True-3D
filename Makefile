OCAMLC   = ocamlfind ocamlc -g
OCAMLMLI = ocamlfind ocamlc
OCAMLOPT = ocamlfind ocamlopt -g
OCAMLDEP = ocamldep.opt
OCAMLDOC = ocamldoc -html

INCLUDES = -I .

LIBS_OPAM = -package unix

SCML = V3.ml avoid.ml scenario.ml main.ml
SCCMO = $(SCML:.ml=.cmo)
SCCMX = $(SCML:.ml=.cmx)
all: opt
byte: orca.out
opt: orca.opt
viz: orca.opt
	./orca.opt
	MPLCONFIGDIR=/tmp/matplotlib python3 viz/plot_3d_avoidance.py --csv sim_trace.csv --out sim_3d.png
orca.out: $(SCCMO)
	$(OCAMLC) -o $@ $(SCCMO)
orca.opt: $(SCCMX)
	$(OCAMLOPT) -linkpkg $(LIBS_OPAM) -o $@ $(SCCMX)
DIRS = sol reduced
.SUFFIXES: .ml .mli .cmi .cmo .cmx
.mli.cmi:
	$(OCAMLMLI) $(INCLUDES) $<
.ml.cmo:
	$(OCAMLC) $(INCLUDES) $(LIBS_OPAM) -c $<
.ml.cmx:
	$(OCAMLOPT) $(INCLUDES) $(LIBS_OPAM) -c $<
.depend:
	$(OCAMLDEP) *.mli *.ml > .depend
include .depend
init:
	mkdir -p $(DIRS)
cleanall: clean #cleandoc
clean:
	rm -f *.cm* *.annot *.o *.out *.opt *.a *~ .depend
