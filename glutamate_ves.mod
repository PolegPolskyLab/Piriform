COMMENT
//****************************//
// Created by Alon Polsky 	//
//    apmega@yahoo.com		//
//		2015				//
//****************************//

quantal release, limited to 32768 release sites (can be expanded)

Example combinations of short term dynamics:
		fast rise, depression	slow rise,stable
newves	=0.005					0.05		
p		=0.2					0.1

ENDCOMMENT

TITLE NMDA-AMPA synapse with depression

NEURON {
	POINT_PROCESS glutamate_ves
	NONSPECIFIC_CURRENT inmda,iampa
	RANGE del,Tspike,Nspike,baseline,doStim
	RANGE local_v,inmda,iampa,e	
	RANGE fascF,fascTau,fascAmp
	RANGE release,numves,p,newVes,t1,SynNum,maxVes
	RANGE gAMPAmax,gNMDAmax,gnmda,gampa,A,B
	GLOBAL n,gama,tau_ampa,tau_nmdaA,tau_nmdaB
	RANGE locx,locy,dend,pos
	GLOBAL Voff,Vset,seed
}

DEFINE maxreleasesites 32768	:limit on the number of release sites

UNITS {
	(nA) 	= (nanoamp)
	(mV)	= (millivolt)
	(nS) 	= (nanomho)
	(mM)    = (milli/liter)
	(mA) = (milliamp)
	(um) = (micron)
}

PARAMETER {
	gNMDAmax  =0.2		(nS):peak conductance
	gAMPAmax  =0.2		(nS)
	SynNum    =1		:number of synapses at the site
	maxVes    =5
	newVes    =0.001	
	p         =0.1		:release probability
	e         =0		(mV)
	tau_nmdaA =50		(ms):nmda	
	tau_nmdaB =2		(ms):nmda
	tau_ampa  =1		(ms)	
	n         =0.25 	(/mM)	
	gama      =0.08 	(/mV) 
	dt 					(ms)
	v					(mV)
	del       =30		(ms)
	Tspike    =10		(ms):time difference between stimuli
	Nspike    =1		:number of stimuli
	baseline  =0.10		:minies rate (Hz)
	doStim	  =1		:do stimulation
	Voff      =0		:0 - voltage dependent 1- voltage independent
	Vset      =-60		:set voltage when voltage independent	
	fascAmp   =0.0		:increase in short term fascilitation (0-no change)
	fascTau   =100		(ms)
	locx=0
	locy=0
	dend=0
	pos=0
	seed=0
}

ASSIGNED {
	inmda		(nA)  
	iampa		(nA)  
	gnmda		(nS)
	local_v	(mV):local voltage
	t1	
	numves[maxreleasesites]
	release
}

STATE {
	A 		(nS)
	B 		(nS)
	gampa 	(nS)
	fascF
}

INITIAL {
    gnmda  =0 
    gampa  =0 
	A      =0
	B      =0
	IF (SynNum>maxreleasesites){SynNum=maxreleasesites}
	FROM i=0 TO SynNum-1{
		numves[i] =maxVes
	}
	t1=0
	set_seed(seed) 
}    

BREAKPOINT {      
	LOCAL count
	SOLVE state METHOD cnexp
	release=0 	
	if (t>t1){
		releasefunc(baseline/1000)
		replenishment()
		t1 =t1+1
	}
	if(doStim==1){
		FROM count=0 TO Nspike-1 {
			IF(at_time(count*Tspike+del)){
				releasefunc(p*(fascF+1))
			}
		}
	}
	A=A+release*gNMDAmax
	B=B+release*gNMDAmax
	gampa=gampa+release*gAMPAmax
	fascF=fascF+release*fascAmp:*(1-fascF)
	
	local_v  =v*(1-Voff)+Vset*Voff	:temp voltage
	gnmda    =(A-B)/(1+n*exp(-gama*local_v) )
	inmda    =(1e-3)*gnmda*(v-e):*SynNum
	iampa    =(1e-3)*gampa*(v-e):*SynNum
	local_v  =v

}

DERIVATIVE state {
	A'=-A/tau_nmdaA
	B'=-B/tau_nmdaB
	gampa'=-gampa/tau_ampa
	fascF'=-fascF/fascTau:+fascAmp*(1-fascF)
}

FUNCTION releasefunc(chance){	:release over all vesicles
	LOCAL i,numrelease,vesnumber,count
	:release=0
	FROM i=0 TO SynNum-1{		
		numrelease=0
		count=(numves[i]-1 )
		FROM vesnumber =0 TO count{
			numrelease=numrelease+(scop_random()<chance)
		}
		release=release+numrelease
		numves[i]=numves[i]-numrelease
		if (numves[i]<0){numves[i]=0}
	}
}

FUNCTION replenishment(){: replenishment over all vesicles	
	LOCAL i,vesnumber,addves,count
	FROM i=0 TO SynNum-1{
		addves=0
		count=(maxVes-numves[i]-1)
		FROM vesnumber=0 TO count {
			if (scop_random()<newVes){addves =addves+1}
		}
		numves[i]=numves[i]+addves
	}
}
