/* Created by Language version: 6.2.0 */
/* NOT VECTORIZED */
#define NRN_VECTORIZED 0
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib_ansi.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__gaba
#define _nrn_initial _nrn_initial__gaba
#define nrn_cur _nrn_cur__gaba
#define _nrn_current _nrn_current__gaba
#define nrn_jacob _nrn_jacob__gaba
#define nrn_state _nrn_state__gaba
#define _net_receive _net_receive__gaba 
#define state state__gaba 
 
#define _threadargscomma_ /**/
#define _threadargsprotocomma_ /**/
#define _threadargs_ /**/
#define _threadargsproto_ /**/
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 static double *_p; static Datum *_ppvar;
 
#define t nrn_threads->_t
#define dt nrn_threads->_dt
#define gmax _p[0]
#define SynNum _p[1]
#define maxVes _p[2]
#define newVes _p[3]
#define p _p[4]
#define e _p[5]
#define del _p[6]
#define Tspike _p[7]
#define Nspike _p[8]
#define baseline _p[9]
#define doStim _p[10]
#define fascAmp _p[11]
#define fascTau _p[12]
#define locx _p[13]
#define locy _p[14]
#define dend _p[15]
#define pos _p[16]
#define igaba _p[17]
#define local_v _p[18]
#define t1 _p[19]
#define numves (_p + 20)
#define release _p[32788]
#define g _p[32789]
#define fascF _p[32790]
#define Dg _p[32791]
#define DfascF _p[32792]
#define _g _p[32793]
#define _nd_area  *_ppvar[0]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_replenishment();
 static double _hoc_releasefunc();
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 extern Prop* nrn_point_prop_;
 static int _pointtype;
 static void* _hoc_create_pnt(_ho) Object* _ho; { void* create_point_process();
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt();
 static double _hoc_loc_pnt(_vptr) void* _vptr; {double loc_point_process();
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(_vptr) void* _vptr; {double has_loc_point();
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(_vptr)void* _vptr; {
 double get_loc_point_process(); return (get_loc_point_process(_vptr));
}
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _p = _prop->param; _ppvar = _prop->dparam;
 }
 static void _hoc_setdata(void* _vptr) { Prop* _prop;
 _prop = ((Point_process*)_vptr)->_prop;
   _setdata(_prop);
 }
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 0,0
};
 static Member_func _member_func[] = {
 "loc", _hoc_loc_pnt,
 "has_loc", _hoc_has_loc,
 "get_loc", _hoc_get_loc_pnt,
 "replenishment", _hoc_replenishment,
 "releasefunc", _hoc_releasefunc,
 0, 0
};
#define replenishment replenishment_gaba
#define releasefunc releasefunc_gaba
 extern double replenishment( );
 extern double releasefunc( double );
 /* declare global and static user variables */
#define Vset Vset_gaba
 double Vset = -60;
#define Voff Voff_gaba
 double Voff = 0;
#define gama gama_gaba
 double gama = 0.08;
#define n n_gaba
 double n = 0.25;
#define seed seed_gaba
 double seed = 0;
#define tau tau_gaba
 double tau = 5;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "tau_gaba", "ms",
 "n_gaba", "/mM",
 "gama_gaba", "/mV",
 "gmax", "nS",
 "e", "mV",
 "del", "ms",
 "Tspike", "ms",
 "fascTau", "ms",
 "g", "nS",
 "igaba", "nA",
 "local_v", "mV",
 0,0
};
 static double delta_t = 0.01;
 static double fascF0 = 0;
 static double g0 = 0;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "tau_gaba", &tau_gaba,
 "n_gaba", &n_gaba,
 "gama_gaba", &gama_gaba,
 "Voff_gaba", &Voff_gaba,
 "Vset_gaba", &Vset_gaba,
 "seed_gaba", &seed_gaba,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(_NrnThread*, _Memb_list*, int);
static void nrn_state(_NrnThread*, _Memb_list*, int);
 static void nrn_cur(_NrnThread*, _Memb_list*, int);
static void  nrn_jacob(_NrnThread*, _Memb_list*, int);
 static void _hoc_destroy_pnt(_vptr) void* _vptr; {
   destroy_point_process(_vptr);
}
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(_NrnThread*, _Memb_list*, int);
static void _ode_matsol(_NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[2]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "6.2.0",
"gaba",
 "gmax",
 "SynNum",
 "maxVes",
 "newVes",
 "p",
 "e",
 "del",
 "Tspike",
 "Nspike",
 "baseline",
 "doStim",
 "fascAmp",
 "fascTau",
 "locx",
 "locy",
 "dend",
 "pos",
 0,
 "igaba",
 "local_v",
 "t1",
 "numves[32768]",
 "release",
 0,
 "g",
 "fascF",
 0,
 0};
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 32794, _prop);
 	/*initialize range parameters*/
 	gmax = 0.2;
 	SynNum = 1;
 	maxVes = 5;
 	newVes = 0.001;
 	p = 0.1;
 	e = -65;
 	del = 30;
 	Tspike = 10;
 	Nspike = 1;
 	baseline = 0.1;
 	doStim = 1;
 	fascAmp = 0;
 	fascTau = 100;
 	locx = 0;
 	locy = 0;
 	dend = 0;
 	pos = 0;
  }
 	_prop->param = _p;
 	_prop->param_size = 32794;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 3, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _GABA_reg() {
	int _vectorized = 0;
  _initlists();
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 0,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
  hoc_register_prop_size(_mechtype, 32794, 3);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 gaba C:/Users/polegpoa/OneDrive - The University of Colorado Denver/simulations/piriform/model/Piriform/GABA.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "NMDA-AMPA synapse with depression";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[2], _dlist1[2];
 static int state(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 () {_reset=0;
 {
   Dg = - g / tau ;
   DfascF = - fascF / fascTau ;
   }
 return _reset;
}
 static int _ode_matsol1 () {
 Dg = Dg  / (1. - dt*( ( - 1.0 ) / tau )) ;
 DfascF = DfascF  / (1. - dt*( ( - 1.0 ) / fascTau )) ;
 return 0;
}
 /*END CVODE*/
 static int state () {_reset=0;
 {
    g = g + (1. - exp(dt*(( - 1.0 ) / tau)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau ) - g) ;
    fascF = fascF + (1. - exp(dt*(( - 1.0 ) / fascTau)))*(- ( 0.0 ) / ( ( - 1.0 ) / fascTau ) - fascF) ;
   }
  return 0;
}
 
double releasefunc (  double _lchance ) {
   double _lreleasefunc;
 double _li , _lnumrelease , _lvesnumber , _lcount ;
 {int  _li ;for ( _li = 0 ; _li <= ((int) SynNum ) - 1 ; _li ++ ) {
     _lnumrelease = 0.0 ;
     _lcount = ( numves [ _li ] - 1.0 ) ;
     {int  _lvesnumber ;for ( _lvesnumber = 0 ; _lvesnumber <= ((int) _lcount ) ; _lvesnumber ++ ) {
       _lnumrelease = _lnumrelease + ( scop_random ( ) < _lchance ) ;
       } }
     release = release + _lnumrelease ;
     numves [ _li ] = numves [ _li ] - _lnumrelease ;
     if ( numves [ _li ] < 0.0 ) {
       numves [ _li ] = 0.0 ;
       }
     } }
   
return _lreleasefunc;
 }
 
static double _hoc_releasefunc(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  releasefunc (  *getarg(1) );
 return(_r);
}
 
double replenishment (  ) {
   double _lreplenishment;
 double _li , _lvesnumber , _laddves , _lcount ;
 {int  _li ;for ( _li = 0 ; _li <= ((int) SynNum ) - 1 ; _li ++ ) {
     _laddves = 0.0 ;
     _lcount = ( maxVes - numves [ _li ] - 1.0 ) ;
     {int  _lvesnumber ;for ( _lvesnumber = 0 ; _lvesnumber <= ((int) _lcount ) ; _lvesnumber ++ ) {
       if ( scop_random ( ) < newVes ) {
         _laddves = _laddves + 1.0 ;
         }
       } }
     numves [ _li ] = numves [ _li ] + _laddves ;
     } }
   
return _lreplenishment;
 }
 
static double _hoc_replenishment(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r =  replenishment (  );
 return(_r);
}
 
static int _ode_count(int _type){ return 2;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 ();
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 2; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 ();
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol_instance1(_threadargs_);
 }}

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
  fascF = fascF0;
  g = g0;
 {
   g = 0.0 ;
   if ( SynNum > 32768.0 ) {
     SynNum = 32768.0 ;
     }
   {int  _li ;for ( _li = 0 ; _li <= ((int) SynNum ) - 1 ; _li ++ ) {
     numves [ _li ] = maxVes ;
     } }
   t1 = 0.0 ;
   set_seed ( seed ) ;
   }
  _sav_indep = t; t = _save;

}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
 initmodel();
}}

static double _nrn_current(double _v){double _current=0.;v=_v;{ {
   double _lcount ;
 release = 0.0 ;
   if ( t > t1 ) {
     releasefunc ( _threadargscomma_ baseline / 1000.0 ) ;
     replenishment ( _threadargs_ ) ;
     t1 = t1 + 1.0 ;
     }
   if ( doStim  == 1.0 ) {
     {int  _lcount ;for ( _lcount = 0 ; _lcount <= ((int) Nspike ) - 1 ; _lcount ++ ) {
       if ( at_time ( nrn_threads, ((double) _lcount ) * Tspike + del ) ) {
         releasefunc ( _threadargscomma_ p * ( fascF + 1.0 ) ) ;
         }
       } }
     }
   g = g + release * gmax ;
   fascF = fascF + release * fascAmp ;
   local_v = v * ( 1.0 - Voff ) + Vset * Voff ;
   igaba = ( 1e-3 ) * g * ( v - e ) ;
   local_v = v ;
   }
 _current += igaba;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _g = _nrn_current(_v + .001);
 	{ _rhs = _nrn_current(_v);
 	}
 _g = (_g - _rhs)/.001;
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
 { error =  state();
 if(error){fprintf(stderr,"at line 99 in file GABA.mod:\n	SOLVE state METHOD cnexp\n"); nrn_complain(_p); abort_run(error);}
 }}}

}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(g) - _p;  _dlist1[0] = &(Dg) - _p;
 _slist1[1] = &(fascF) - _p;  _dlist1[1] = &(DfascF) - _p;
_first = 0;
}
