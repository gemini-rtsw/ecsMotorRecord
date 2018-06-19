/*******************************************************************************
Support for keeping track of which record fields have been changed, so we can
eliminate redundant db_post_events() without having to think, and without having
to keep lots of "last value of field xxx" fields in the record.  The idea is
to say...

	MARK(M_XXXX);

when you mean...

	db_post_events(pmr, &pmr->xxxx, monitor_mask);

Before leaving, you have to call post_MARKed_fields() to actually post the
field to all listeners.  monitor() does this.

	--- NOTE WELL ---
	The macros below assume that the variable "pmr" exists and points to a
	motor record, like so:
		struct ecsMotorRecord *pmr;
	No check is made in this code to ensure that this really is true.
*******************************************************************************/
#define M_VAL		0x00000001
#define M_VELO		0x00000002
#define M_MODE		0x00000004
#define M_SIMM		0x00000008
#define	M_LLS		0x00000010
#define	M_HLS		0x00000020
#define	M_FLT		0x00000040
#define	M_MESS		0x00000080
#define M_RRBV          0x00000100
#define M_MPOS          0x00000200
#define M_MSTA		0x00000400
#define M_HSTA		0x00000800
#define M_DMOV		0x00001000
#define	M_MOVN		0x00002000
#define	M_MIP		0x00004000
#define	M_TDIR		0x00008000
#define	M_HLM		0x00010000
#define	M_LLM		0x00020000
#define	M_LVIO		0x00040000
#define M_DSTA          0x00080000
#define M_STOP          0x00100000
#define M_MDBD          0x00200000
#define M_DBUG          0x00400000
#define M_HINP          0x00800000
#define M_PDFB          0x01000000

/* internal flags for async communications */
#define M_ERROR         0x10000000
#define M_WRITE         0x20000000
#define M_TIMEOUT       0x40000000
#define M_RESCAN        0x80000000

/* funky macros to handle the above */
#define MARK(a)	        pmr->mmap |= (a);
#define MARKED(a)	pmr->mmap&(a)
#define UNMARK(a)	pmr->mmap &= ~(a);
#define UNMARK_ALL	pmr->mmap = 0;

