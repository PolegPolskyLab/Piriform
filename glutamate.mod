COMMENT
//****************************//
// Created by Alon Polsky 	//
//    apmega@yahoo.com		//
//		2010			//
//****************************//
ENDCOMMENT

TITLE NMDA synapse with depression

NEURON {
	POINT_PROCESS glutamate
	NONSPECIFIC_CURRENT inmda,iampa
	RANGE del,Tspike,Nspike
	RANGE e ,gAMPAmax,gNMDAmax,local_v,inmda,iampa

	RANGE gnmda,gampa,dend,pos,locx,locy

	GLOBAL n, gama,tau_ampa
	GLOBAL tau1,tau2
	GLOBAL Voff,Vset

}

UNITS {
	(nA) 	= (nanoamp)
	(mV)	= (millivolt)
	(nS) 	= (nanomho)
	(mM)    = (milli/liter)
        F	= 96480 (coul)
        R       = 8.314 (volt-coul/degC)
 	PI = (pi) (1)
	(mA) = (milliamp)
	(um) = (micron)

}

PARAMETER {
	gNMDAmax=1	(nS)
	gAMPAmax=1	(nS)
	e= 0.0	(mV)
	tau1=50	(ms)	
	tau2=2	(ms)	
	tau_ampa=1	(ms)	
	n=0.25 	(/mM)	
	gama=0.08 	(/mV) 
	dt (ms)
	v		(mV)
	del=30	(ms)
	Tspike=10	(ms)
	Nspike=1

	dend=0
	pos=0
	locx=0
	locy=0
	Voff      =0		:0 - voltage dependent 1- voltage independent
	Vset      =-60		:set voltage when voltage independent			
}

ASSIGNED {
	inmda		(nA)  
	iampa		(nA)  
	gnmda		(nS)
	local_v	(mV):local voltage

}
STATE {
	A 		(nS)
	B 		(nS)
	gampa 	(nS)

}

INITIAL {
    gnmda=0 
    gampa=0 
	A=0
	B=0


}    

BREAKPOINT {  
    
	LOCAL count
	SOLVE state METHOD cnexp
	FROM count=0 TO Nspike-1 {
		IF(at_time(count*Tspike+del)){
			state_discontinuity( A, A+ gNMDAmax)
			state_discontinuity( B, B+ gNMDAmax)
			state_discontinuity( gampa, gampa+ gAMPAmax)

		}
	}
	local_v  =v*(1-Voff)+Vset*Voff	:temp voltage
	gnmda    =(A-B)/(1+n*exp(-gama*local_v) )
	
	inmda =(1e-3)* gnmda  * (v-e)
	iampa= (1e-3)*gampa* (v- e)
	:local_v=v


}

DERIVATIVE state {
	A'=-A/tau1
	B'=-B/tau2
	gampa'=-gampa/tau_ampa
}
