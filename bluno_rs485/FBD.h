


/*
    In order to prevent the shake of Key input signal, I use time delay function block.
    It is called that TON FBD in IEC61131-5 standard.

    Only IN signal is maintained for specified time, Q output become 1.
    Otherwise Q is 0.
*/
typedef struct timeronblock TON;
struct timeronblock
{
  unsigned IN: 1; // IN option
  unsigned Q: 1; // Output
  unsigned long PT; // Set point
  unsigned long ET; // Elapsed time
};

/*
 *  
 *  In order to detect rising edge of input signal, I use Rising trigger function block.
 *  It is called that R_trig FBD in IEC61131-5.
 *  
 *  function: I have used it in order to detect key pressed signal so that I noticed to program that mode is changed.
*/
typedef struct RisingTrg Rtrg;
struct RisingTrg
{
    unsigned IN : 1;
    unsigned PRE : 1;
    unsigned Q : 1;
};

// When any condition is true, it returns true
void TONFunc(TON *pTP)
{
    if(pTP->IN)
    {
        if((pTP->ET + pTP->PT) <= millis())
            pTP->Q = 1;  
    }
    else
    {
        pTP->ET = millis();
        pTP->Q = 0;
    }
}

// It should be used with TONFunc together
void RTrgFunc(Rtrg *pTrg)
{
    pTrg->Q = 0;
    if(pTrg->IN != pTrg->PRE)
    {
        pTrg->PRE = pTrg->IN;
        if(pTrg->PRE == 1)
        {
            pTrg->Q = 1;
        }    
    }
}

