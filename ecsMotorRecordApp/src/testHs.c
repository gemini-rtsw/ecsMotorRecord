#include <stdio.h>
#include "ecsMotorRecordDefs.h"

#define TRUE		1
#define M_HSTA		0x00000800
#define MARK(a)		pmr->mmap |= (a);

/* handshake bit masks */
#define INPUT_MASK(vms) ((vms)?\
			(PWR_BIT|PWR_ACK_BIT|DRV_ACK_BIT|POS_ACK_BIT|IN_POS_BIT|VEL_ACK_BIT):\
			(PWR_BIT|PWR_ACK_BIT|DRV_ACK_BIT|POS_ACK_BIT|IN_POS_BIT))
#define OUTPUT_MASK(vms) ((vms)?\
			 (NEW_POS_BIT|NEW_VEL_BIT|DRV_BIT):\
			 (NEW_POS_BIT|DRV_BIT))

/* Fake motor record
 */
struct ecsMotorRecord {
    int		hinp;
    int		hsta;
    int		mmap;
    int		pp;
    int		vms;
};

/*
 * Function setHandshakeBits
 * 
 * Set one or more bits in the output handshake.
 * This routine is intended to be called from the state machine functions.
 *
 */
static void setHandshakeBits (struct ecsMotorRecord *pmr, unsigned int pattern)
{
    long input_mask, output_mask;

    input_mask = INPUT_MASK(pmr->vms);
    output_mask = OUTPUT_MASK(pmr->vms);

    printf ("\nsetHandshakeBits1: im=0x%lx om=0x%lx\n", input_mask, output_mask);
    printf ("setHandshakeBits2: hi=0x%x, ho=0x%x, p=0x%x\n", pmr->hinp, pmr->hsta, pattern);

    pmr->hsta = (pmr->hinp & input_mask) | ((pmr->hsta | pattern) & output_mask);
    pmr->pp = TRUE;
    MARK(M_HSTA);

    printf ("setHandshakeBits3: hi=0x%x, ho=0x%x\n", pmr->hinp, pmr->hsta);
}

/*
 * Function resetHandshakeBits
 * 
 * Reset (clear) one or more bits in the output handshake.
 * This routine is intended to be called from the state machine functions.
 *
 */
static void resetHandshakeBits (struct ecsMotorRecord *pmr, unsigned int pattern)
{
    long input_mask, output_mask;

    input_mask = INPUT_MASK(pmr->vms);
    output_mask = OUTPUT_MASK(pmr->vms);

    printf ("\nresetHandshakeBits1: im=0x%x om=0x%x\n", input_mask, output_mask);
    printf ("resetHandshakeBits2: hi=0x%x, ho=0x%x, p=0x%x\n", pmr->hinp, pmr->hsta, pattern);
    
    pmr->hsta = (pmr->hinp & input_mask) | ((pmr->hsta & ~pattern) & output_mask);
    pmr->pp = TRUE;
    MARK(M_HSTA);

    printf ("resetHandshakeBits3: hi=0x%x, ho=0x%x\n", pmr->hinp, pmr->hsta);
}

/*
 * Function updateInputBits
 * 
 * Update changes in the handshake input bits into the handshake output.
 * This routine is intended to be called from the scan task when a change in one
 * of the input handshake bits (HINP) is detected and needs to be propagated to
 * the output handshake (HSTA) to keep both words syncronized.
 *
 */
static void updateInputBits (struct ecsMotorRecord *pmr)
{
    long input_mask, output_mask;

    input_mask = INPUT_MASK(pmr->vms);
    output_mask = OUTPUT_MASK(pmr->vms);

    printf ("\nupdateHandShakeBits1: im=0x%lx om=0x%lx\n", input_mask, output_mask);
    printf ("updateHandShakeBits2: hi=0x%x, ho=0x%x\n", pmr->hinp, pmr->hsta);

    pmr->hsta = (pmr->hinp & input_mask) | (pmr->hsta & output_mask);
    pmr->pp = TRUE;
    MARK(M_HSTA);

    printf ("updateHandShakeBits3: hi=0x%x, ho=0x%x\n", pmr->hinp, pmr->hsta);
}

void main() {

    struct ecsMotorRecord mr;
    struct ecsMotorRecord *pmr = &mr;

    pmr->pp = 0;
    pmr->vms = 0;

    /* power on and in position */
    pmr->hinp = 0x9;
    pmr->hsta = 0x9;

    /* new position */
    setHandshakeBits(pmr, NEW_POS_BIT);

    /* new pos received at hinp along with pos ack */
    pmr->hinp = 0x49;
    updateInputBits(pmr);

    /* reset new pos */
    resetHandshakeBits(pmr, NEW_POS_BIT);

    /* new pos reset is received */
    pmr->hinp = 0x9;
    updateInputBits(pmr);

    /* drive enable */
    setHandshakeBits(pmr, DRV_BIT);

    /* hinp receives drv ack and pos ack reset */
    pmr->hinp = 0x23;
    updateInputBits(pmr);

    /* move in progress */

    /* hinp receives in position */
    pmr->hinp = 0x29;
    updateInputBits(pmr);

    /* reset drive enable */
    resetHandshakeBits(pmr, DRV_BIT);

    /* hinp receives a change, bur drv enable is still active */
     setHandshakeBits(pmr, pmr->hinp); /* bug! */
    //updateInputBits(pmr);
}
