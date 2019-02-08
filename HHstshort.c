/* Created by Language version: 6.2.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
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
 
#define nrn_init _nrn_init__HH
#define _nrn_initial _nrn_initial__HH
#define nrn_cur _nrn_cur__HH
#define _nrn_current _nrn_current__HH
#define nrn_jacob _nrn_jacob__HH
#define nrn_state _nrn_state__HH
#define _net_receive _net_receive__HH 
#define rates rates__HH 
#define states states__HH 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define gnabar _p[0]
#define gkbar _p[1]
#define gkmbar _p[2]
#define gleak _p[3]
#define eleak _p[4]
#define NFleak _p[5]
#define ileak _p[6]
#define gna _p[7]
#define gk _p[8]
#define Nna _p[9]
#define Nk _p[10]
#define Nkm _p[11]
#define m_exp _p[12]
#define h_exp _p[13]
#define n_exp _p[14]
#define km_exp _p[15]
#define m_inf _p[16]
#define h_inf _p[17]
#define n_inf _p[18]
#define km_inf _p[19]
#define noise_zn (_p + 20)
#define noise_zm (_p + 23)
#define noise_zkm _p[29]
#define var_zn (_p + 30)
#define var_zm (_p + 33)
#define var_zkm _p[39]
#define tau_m _p[40]
#define tau_h _p[41]
#define tau_n _p[42]
#define tau_km _p[43]
#define tau_zn (_p + 44)
#define tau_zm (_p + 47)
#define tau_zkm _p[53]
#define mu_zn (_p + 54)
#define mu_zm (_p + 57)
#define mu_zkm _p[63]
#define m _p[64]
#define h _p[65]
#define n _p[66]
#define nm _p[67]
#define zn (_p + 68)
#define zm (_p + 71)
#define zkm _p[77]
#define zleak _p[78]
#define Dm _p[79]
#define Dh _p[80]
#define Dn _p[81]
#define Dnm _p[82]
#define Dzn (_p + 83)
#define Dzm (_p + 86)
#define Dzkm _p[92]
#define Dzleak _p[93]
#define ina _p[94]
#define ikdr _p[95]
#define ikm _p[96]
#define ik _p[97]
#define gkm _p[98]
#define ena _p[99]
#define ek _p[100]
#define v _p[101]
#define _g _p[102]
#define _ion_ena	*_ppvar[0]._pval
#define _ion_ina	*_ppvar[1]._pval
#define _ion_dinadv	*_ppvar[2]._pval
#define _ion_ek	*_ppvar[3]._pval
#define _ion_ik	*_ppvar[4]._pval
#define _ion_dikdv	*_ppvar[5]._pval
#define area	*_ppvar[6]._pval
 
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
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 extern double celsius;
 /* declaration of user functions */
 static void _hoc_mulnoise(void);
 static void _hoc_numchan(void);
 static void _hoc_rates(void);
 static void _hoc_states(void);
 static void _hoc_vtrap(void);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata() {
 Prop *_prop, *hoc_getdata_range(int);
 _prop = hoc_getdata_range(_mechtype);
   _setdata(_prop);
 hoc_retpushx(1.);
}
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 "setdata_HH", _hoc_setdata,
 "mulnoise_HH", _hoc_mulnoise,
 "numchan_HH", _hoc_numchan,
 "rates_HH", _hoc_rates,
 "states_HH", _hoc_states,
 "vtrap_HH", _hoc_vtrap,
 0, 0
};
#define mulnoise mulnoise_HH
#define numchan numchan_HH
#define vtrap vtrap_HH
 extern double mulnoise( _threadargsprotocomma_ double , double , double );
 extern double numchan( _threadargsprotocomma_ double );
 extern double vtrap( _threadargsprotocomma_ double , double );
 /* declare global and static user variables */
#define NF NF_HH
 double NF = 1;
#define gamma_km gamma_km_HH
 double gamma_km = 10;
#define gamma_k gamma_k_HH
 double gamma_k = 10;
#define gamma_na gamma_na_HH
 double gamma_na = 10;
#define hfast hfast_HH
 double hfast = 0.3;
#define hslow hslow_HH
 double hslow = 100;
#define seed seed_HH
 double seed = 1;
#define taukm taukm_HH
 double taukm = 1;
#define vshift vshift_HH
 double vshift = 0;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "gamma_na_HH", "pS",
 "gamma_k_HH", "pS",
 "gamma_km_HH", "pS",
 "gnabar_HH", "S/cm2",
 "gkbar_HH", "S/cm2",
 "gkmbar_HH", "S/cm2",
 "gleak_HH", "S/cm2",
 "eleak_HH", "mV",
 "ileak_HH", "mA/cm2",
 "gna_HH", "S/cm2",
 "gk_HH", "S/cm2",
 "Nna_HH", "1",
 "Nk_HH", "1",
 "Nkm_HH", "1",
 "tau_m_HH", "ms",
 "tau_h_HH", "ms",
 "tau_n_HH", "ms",
 "tau_km_HH", "ms",
 0,0
};
 static double delta_t = 0.01;
 static double h0 = 0;
 static double m0 = 0;
 static double nm0 = 0;
 static double n0 = 0;
 static double zleak0 = 0;
 static double zkm0 = 0;
 static double zm0 = 0;
 static double zn0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "gamma_na_HH", &gamma_na_HH,
 "gamma_k_HH", &gamma_k_HH,
 "gamma_km_HH", &gamma_km_HH,
 "seed_HH", &seed_HH,
 "vshift_HH", &vshift_HH,
 "taukm_HH", &taukm_HH,
 "NF_HH", &NF_HH,
 "hslow_HH", &hslow_HH,
 "hfast_HH", &hfast_HH,
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
 
static int _ode_count(int);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "6.2.0",
"HH",
 "gnabar_HH",
 "gkbar_HH",
 "gkmbar_HH",
 "gleak_HH",
 "eleak_HH",
 "NFleak_HH",
 0,
 "ileak_HH",
 "gna_HH",
 "gk_HH",
 "Nna_HH",
 "Nk_HH",
 "Nkm_HH",
 "m_exp_HH",
 "h_exp_HH",
 "n_exp_HH",
 "km_exp_HH",
 "m_inf_HH",
 "h_inf_HH",
 "n_inf_HH",
 "km_inf_HH",
 "noise_zn_HH[3]",
 "noise_zm_HH[6]",
 "noise_zkm_HH",
 "var_zn_HH[3]",
 "var_zm_HH[6]",
 "var_zkm_HH",
 "tau_m_HH",
 "tau_h_HH",
 "tau_n_HH",
 "tau_km_HH",
 "tau_zn_HH[3]",
 "tau_zm_HH[6]",
 "tau_zkm_HH",
 "mu_zn_HH[3]",
 "mu_zm_HH[6]",
 "mu_zkm_HH",
 0,
 "m_HH",
 "h_HH",
 "n_HH",
 "nm_HH",
 "zn_HH[3]",
 "zm_HH[6]",
 "zkm_HH",
 "zleak_HH",
 0,
 0};
 extern Node* nrn_alloc_node_;
 static Symbol* _na_sym;
 static Symbol* _k_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 103, _prop);
 	/*initialize range parameters*/
 	gnabar = 0.12;
 	gkbar = 0.036;
 	gkmbar = 0.002;
 	gleak = 1e-005;
 	eleak = -60;
 	NFleak = 1;
 	_prop->param = _p;
 	_prop->param_size = 103;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 7, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 	_ppvar[6]._pval = &nrn_alloc_node_->_area; /* diam */
 prop_ion = need_memb(_na_sym);
 nrn_promote(prop_ion, 0, 1);
 	_ppvar[0]._pval = &prop_ion->param[0]; /* ena */
 	_ppvar[1]._pval = &prop_ion->param[3]; /* ina */
 	_ppvar[2]._pval = &prop_ion->param[4]; /* _ion_dinadv */
 prop_ion = need_memb(_k_sym);
 nrn_promote(prop_ion, 0, 1);
 	_ppvar[3]._pval = &prop_ion->param[0]; /* ek */
 	_ppvar[4]._pval = &prop_ion->param[3]; /* ik */
 	_ppvar[5]._pval = &prop_ion->param[4]; /* _ion_dikdv */
 
}
 static void _initlists();
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _HHstshort_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("na", -10000.);
 	ion_reg("k", -10000.);
 	_na_sym = hoc_lookup("na_ion");
 	_k_sym = hoc_lookup("k_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
  hoc_register_prop_size(_mechtype, 103, 7);
  hoc_register_dparam_semantics(_mechtype, 0, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "na_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 6, "area");
 	hoc_register_cvode(_mechtype, _ode_count, 0, 0, 0);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 HH C:/Users/polegpoa/OneDrive - The University of Colorado Denver/simulations/piriform/model/Piriform/HHstshort.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "Stochastic Hodgkin and Huxley model & M-type potassium & T-and L-type Calcium channels incorporating channel noise .";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int rates(_threadargsprotocomma_ double);
static int states(_threadargsproto_);
 
static int  states ( _threadargsproto_ ) {
   rates ( _threadargscomma_ v + vshift ) ;
   m = m + m_exp * ( m_inf - m ) ;
   h = h + h_exp * ( h_inf - h ) ;
   n = n + n_exp * ( n_inf - n ) ;
   nm = nm + km_exp * ( km_inf - nm ) ;
   
/*VERBATIM*/
    return 0;
  return 0; }
 
static void _hoc_states(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r = 1.;
 states ( _p, _ppvar, _thread, _nt );
 hoc_retpushx(_r);
}
 
static int  rates ( _threadargsprotocomma_ double _lvm ) {
   double _la , _lb , _lm3_inf , _ln4_inf , _lsum , _lone_minus_m , _lone_minus_h , _lone_minus_n , _li ;
  _la = - .6 * vtrap ( _threadargscomma_ ( _lvm + 30.0 ) , - 10.0 ) ;
   _lb = 20.0 * ( exp ( ( - 1.0 * ( _lvm + 55.0 ) ) / 18.0 ) ) ;
   tau_m = 1.0 / ( _la + _lb ) ;
   m_inf = _la * tau_m ;
   _lone_minus_m = 1. - m_inf ;
   _lm3_inf = m_inf * m_inf * m_inf ;
   _la = 0.4 * ( exp ( ( - 1.0 * ( _lvm + 50.0 ) ) / 20.0 ) ) ;
   _lb = 6.0 / ( 1.0 + exp ( - 0.1 * ( _lvm + 20.0 ) ) ) ;
   tau_h = hslow / ( ( 1.0 + exp ( ( _lvm + 30.0 ) / 4.0 ) ) + ( exp ( - ( _lvm + 50.0 ) / 2.0 ) ) ) + hfast ;
   h_inf = 1.0 / ( 1.0 + exp ( ( _lvm + 44.0 ) / 4.0 ) ) ;
   _lone_minus_h = 1. - h_inf ;
   tau_zm [ 0 ] = tau_h ;
   tau_zm [ 1 ] = tau_m ;
   tau_zm [ 2 ] = tau_m / 2.0 ;
   tau_zm [ 3 ] = tau_m / 3.0 ;
   tau_zm [ 4 ] = tau_m * tau_h / ( tau_m + tau_h ) ;
   tau_zm [ 5 ] = tau_m * tau_h / ( tau_m + 2.0 * tau_h ) ;
   tau_zm [ 6 ] = tau_m * tau_h / ( tau_m + 3.0 * tau_h ) ;
   var_zm [ 0 ] = 1.0 / numchan ( _threadargscomma_ Nna ) * _lm3_inf * _lm3_inf * h_inf * _lone_minus_h ;
   var_zm [ 1 ] = 3.0 / numchan ( _threadargscomma_ Nna ) * _lm3_inf * m_inf * m_inf * h_inf * h_inf * _lone_minus_m ;
   var_zm [ 2 ] = 3.0 / numchan ( _threadargscomma_ Nna ) * _lm3_inf * m_inf * h_inf * h_inf * _lone_minus_m * _lone_minus_m ;
   var_zm [ 3 ] = 1.0 / numchan ( _threadargscomma_ Nna ) * _lm3_inf * h_inf * h_inf * _lone_minus_m * _lone_minus_m * _lone_minus_m ;
   var_zm [ 4 ] = 3.0 / numchan ( _threadargscomma_ Nna ) * _lm3_inf * m_inf * m_inf * h_inf * _lone_minus_m * _lone_minus_h ;
   var_zm [ 5 ] = 3.0 / numchan ( _threadargscomma_ Nna ) * _lm3_inf * m_inf * h_inf * _lone_minus_m * _lone_minus_m * _lone_minus_h ;
   var_zm [ 6 ] = 1.0 / numchan ( _threadargscomma_ Nna ) * _lm3_inf * h_inf * _lone_minus_m * _lone_minus_m * _lone_minus_m * _lone_minus_h ;
   {int  _li ;for ( _li = 0 ; _li <= 6 ; _li ++ ) {
     mu_zm [ _li ] = exp ( - dt / tau_zm [ _li ] ) ;
     noise_zm [ _li ] = sqrt ( var_zm [ _li ] * ( 1.0 - mu_zm [ _li ] * mu_zm [ _li ] ) ) * normrand ( 0.0 , NF ) ;
     zm [ _li ] = zm [ _li ] * mu_zm [ _li ] + noise_zm [ _li ] ;
     } }
   _la = - 0.02 * vtrap ( _threadargscomma_ ( _lvm + 40.0 ) , - 10.0 ) ;
   _lb = 0.4 * ( exp ( ( - 1.0 * ( _lvm + 50.0 ) ) / 80.0 ) ) ;
   tau_n = 1.0 / ( _la + _lb ) ;
   n_inf = _la * tau_n ;
   _lone_minus_n = 1. - n_inf ;
   _ln4_inf = n_inf * n_inf * n_inf * n_inf ;
   tau_zn [ 0 ] = tau_n ;
   tau_zn [ 1 ] = tau_n / 2.0 ;
   tau_zn [ 2 ] = tau_n / 3.0 ;
   tau_zn [ 3 ] = tau_n / 4.0 ;
   var_zn [ 0 ] = 4.0 / numchan ( _threadargscomma_ Nk ) * _ln4_inf * n_inf * n_inf * n_inf * _lone_minus_n ;
   var_zn [ 1 ] = 6.0 / numchan ( _threadargscomma_ Nk ) * _ln4_inf * n_inf * n_inf * _lone_minus_n * _lone_minus_n ;
   var_zn [ 2 ] = 4.0 / numchan ( _threadargscomma_ Nk ) * _ln4_inf * n_inf * _lone_minus_n * _lone_minus_n * _lone_minus_n ;
   var_zn [ 3 ] = 1.0 / numchan ( _threadargscomma_ Nk ) * _ln4_inf * _lone_minus_n * _lone_minus_n * _lone_minus_n * _lone_minus_n ;
   {int  _li ;for ( _li = 0 ; _li <= 3 ; _li ++ ) {
     mu_zn [ _li ] = exp ( - dt / tau_zn [ _li ] ) ;
     noise_zn [ _li ] = sqrt ( var_zn [ _li ] * ( 1.0 - mu_zn [ _li ] * mu_zn [ _li ] ) ) * normrand ( 0.0 , NF ) ;
     zn [ _li ] = zn [ _li ] * mu_zn [ _li ] + noise_zn [ _li ] ;
     } }
   _la = - .001 / taukm * vtrap ( _threadargscomma_ ( _lvm + 30.0 ) , - 9.0 ) ;
   _lb = .001 / taukm * vtrap ( _threadargscomma_ ( _lvm + 30.0 ) , 9.0 ) ;
   tau_km = 1.0 / ( _la + _lb ) ;
   km_inf = _la * tau_km ;
   tau_zkm = tau_km ;
   var_zkm = km_inf * ( 1.0 - km_inf ) / numchan ( _threadargscomma_ Nkm ) ;
   mu_zkm = exp ( - dt / tau_zkm ) ;
   noise_zkm = sqrt ( var_zkm * ( 1.0 - mu_zkm * mu_zkm ) ) * normrand ( 0.0 , NF ) ;
   zkm = zkm * mu_zkm + noise_zkm ;
   zleak = normrand ( 0.0 , NFleak ) ;
   m_exp = 1.0 - exp ( - dt / tau_m ) ;
   h_exp = 1.0 - exp ( - dt / tau_h ) ;
   n_exp = 1.0 - exp ( - dt / tau_n ) ;
   km_exp = 1.0 - exp ( - dt / tau_km ) ;
     return 0; }
 
static void _hoc_rates(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r = 1.;
 rates ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
double vtrap ( _threadargsprotocomma_ double _lx , double _ly ) {
   double _lvtrap;
 if ( fabs ( exp ( _lx / _ly ) - 1.0 ) < 1e-6 ) {
     _lvtrap = _ly * ( 1.0 - _lx / _ly / 2.0 ) ;
     }
   else {
     _lvtrap = _lx / ( exp ( _lx / _ly ) - 1.0 ) ;
     }
   
return _lvtrap;
 }
 
static void _hoc_vtrap(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  vtrap ( _p, _ppvar, _thread, _nt, *getarg(1) , *getarg(2) );
 hoc_retpushx(_r);
}
 
double mulnoise ( _threadargsprotocomma_ double _lmean , double _lsd , double _lpower ) {
   double _lmulnoise;
 double _li , _lavg ;
 _lavg = 1.0 ;
   {int  _li ;for ( _li = 1 ; _li <= ((int) _lpower ) ; _li ++ ) {
     _lavg = _lavg * normrand ( _lmean , _lsd ) ;
     } }
   _lmulnoise = _lavg ;
   
return _lmulnoise;
 }
 
static void _hoc_mulnoise(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  mulnoise ( _p, _ppvar, _thread, _nt, *getarg(1) , *getarg(2) , *getarg(3) );
 hoc_retpushx(_r);
}
 
double numchan ( _threadargsprotocomma_ double _lNchannels ) {
   double _lnumchan;
 if ( _lNchannels > 0.0 ) {
     _lnumchan = ( _lNchannels ) ;
     }
   else {
     _lnumchan = 1.0 ;
     }
   
return _lnumchan;
 }
 
static void _hoc_numchan(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r =  numchan ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
static int _ode_count(int _type){ hoc_execerror("HH", "cannot be used with CVODE"); return 0;}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_na_sym, _ppvar, 0, 0);
   nrn_update_ion_pointer(_na_sym, _ppvar, 1, 3);
   nrn_update_ion_pointer(_na_sym, _ppvar, 2, 4);
   nrn_update_ion_pointer(_k_sym, _ppvar, 3, 0);
   nrn_update_ion_pointer(_k_sym, _ppvar, 4, 3);
   nrn_update_ion_pointer(_k_sym, _ppvar, 5, 4);
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  h = h0;
  m = m0;
  n = n0;
  nm = nm0;
  zleak = zleak0;
  zkm = zkm0;
 for (_i=0; _i<6; _i++) zm[_i] = zm0;
 for (_i=0; _i<3; _i++) zn[_i] = zn0;
 {
   Nna = ceil ( ( ( 1e-8 ) * area ) * ( gnabar ) / ( ( 1e-12 ) * gamma_na ) ) ;
   Nk = ceil ( ( ( 1e-8 ) * area ) * ( gkbar ) / ( ( 1e-12 ) * gamma_k ) ) ;
   Nkm = ceil ( ( ( 1e-8 ) * area ) * ( gkmbar ) / ( ( 1e-12 ) * gamma_km ) ) ;
   rates ( _threadargscomma_ v ) ;
   m = m_inf ;
   h = h_inf ;
   n = n_inf ;
   nm = km_inf ;
   zn [ 0 ] = 0.0 ;
   zn [ 1 ] = 0.0 ;
   zn [ 2 ] = 0.0 ;
   zn [ 3 ] = 0.0 ;
   zm [ 0 ] = 0. ;
   zm [ 1 ] = 0. ;
   zm [ 2 ] = 0. ;
   zm [ 3 ] = 0. ;
   zm [ 4 ] = 0. ;
   zm [ 5 ] = 0. ;
   zm [ 6 ] = 0. ;
   zkm = 0.0 ;
   zleak = 0.0 ;
   set_seed ( seed ) ;
   }
 
}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
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
  ena = _ion_ena;
  ek = _ion_ek;
 initmodel(_p, _ppvar, _thread, _nt);
  }
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   gna = gnabar * ( m * m * m * h + ( zm [ 0 ] + zm [ 1 ] + zm [ 2 ] + zm [ 3 ] + zm [ 4 ] + zm [ 5 ] + zm [ 6 ] ) ) ;
   ina = gna * ( v - ena ) ;
   gk = gkbar * ( n * n * n * n + ( zn [ 0 ] + zn [ 1 ] + zn [ 2 ] + zn [ 3 ] ) ) ;
   ikdr = gk * ( v - ek ) ;
   gkm = gkmbar * ( nm + zkm ) ;
   ikm = gkm * ( v - ek ) ;
   ik = ikm + ikdr ;
   ileak = gleak * ( 1.0 + zleak ) * ( v - eleak ) ;
   }
 _current += ina;
 _current += ik;
 _current += ileak;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
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
  ena = _ion_ena;
  ek = _ion_ek;
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ double _dik;
 double _dina;
  _dina = ina;
  _dik = ik;
 _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
  _ion_dinadv += (_dina - ina)/.001 ;
  _ion_dikdv += (_dik - ik)/.001 ;
 	}
 _g = (_g - _rhs)/.001;
  _ion_ina += ina ;
  _ion_ik += ik ;
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}
 
}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
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
 
}
 
}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
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
  ena = _ion_ena;
  ek = _ion_ek;
 {  { states(_p, _ppvar, _thread, _nt); }
  }  }}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif
