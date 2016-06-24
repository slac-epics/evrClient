#ifndef HI_RES_TIME_STUB_H
#define HI_RES_TIME_STUB_H

#ifdef	__cplusplus
extern "C" {
#endif

typedef		long long		t_HiResTime;

t_HiResTime	GetHiResTicks();

#ifdef	__cplusplus
}
#endif	/*	__cplusplus	*/


/*
 * Macro definitions for reading the timestamp counter
 * Add processor specific variants of the read_tsc() macro as needed and available
 * If read_tsc() is not defined for your cpu, a version of GetHiResTicks() is available
 * in the diagTimer module as a function that provides a backup using gettimeofday()
 */
#if	defined(__x86_64__)

#define		read_tsc( tscVal	)	     					\
do										      				\
{	/*	=A is for 32 bit mode reading 64 bit integer */		\
	unsigned long	tscHi, tscLo;      						\
	__asm__ volatile("rdtsc" : "=a" (tscLo), "=d" (tscHi) );    \
	tscVal	=  (t_HiResTime)( tscHi ) << 32;      			\
	tscVal	|= (t_HiResTime)( tscLo );      				\
}	while ( 0 );

/*	end of defined(__x86_64__)	*/
#elif	defined(__i386__)

#define	read_tsc(tscVal)	__asm__ volatile( "rdtsc": "=A" (tscVal) )

/*	end of defined(__i386__)	*/
#elif	defined(mpc7455) || defined(__PPC__)

#define		read_tsc( tscVal	)	     					\
do										      				\
{											      			\
	unsigned long	tscHi, tscHi_old, tscLo;				\
	do														\
	{														\
		__asm__ volatile("mftbu %0" : "=r" (tscHi_old));	\
		__asm__ volatile("mftb  %0" : "=r" (tscLo));		\
		__asm__ volatile("mftbu %0" : "=r" (tscHi));		\
	}	while (tscHi_old != tscHi);							\
	tscVal	=  (t_HiResTime)( tscHi ) << 32;      			\
	tscVal	|= (t_HiResTime)( tscLo );      				\
}	while ( 0 );

#endif	/*	end of	defined(mpc7455)	*/

#ifdef	read_tsc
extern __inline__ t_HiResTime GetHiResTicks()
{
	t_HiResTime		tscVal	= 0;
	read_tsc( tscVal );
	return tscVal;
}
#endif	/*	read_tsc	*/

#ifdef	__cplusplus
//	C++ functions

#endif	/*	__cplusplus	*/

#endif	/*  HI_RES_TIME_STUB_H	*/
