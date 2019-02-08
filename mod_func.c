#include <stdio.h>
#include "hocdec.h"
#define IMPORT extern __declspec(dllimport)
IMPORT int nrnmpi_myid, nrn_nobanner_;

extern void _GABA_reg();
extern void _HHstshort_reg();
extern void _glutamate_reg();
extern void _glutamate_ves_reg();

modl_reg(){
	//nrn_mswindll_stdio(stdin, stdout, stderr);
    if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
	fprintf(stderr, "Additional mechanisms from files\n");

fprintf(stderr," GABA.mod");
fprintf(stderr," HHstshort.mod");
fprintf(stderr," glutamate.mod");
fprintf(stderr," glutamate_ves.mod");
fprintf(stderr, "\n");
    }
_GABA_reg();
_HHstshort_reg();
_glutamate_reg();
_glutamate_ves_reg();
}
