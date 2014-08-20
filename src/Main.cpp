#include <iostream>
#include <stdio.h> //for printf
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <imar/imar.hpp>

/**
* SIGNAL      ID   DEFAULT  DESCRIPTION
* ===============================================
*	SIGHUP 		1    Termin.  Hang up on controlling terminal
*  SIGINT			2    Termin.  Interrupt. Generated when we enter CNRTL-C and it is delivered
*								to all processes/threads associated to the current terminal. If generated
*								with kill, it is delivered to only one process/thread.
*  SIGQUIT      3    Core     Generated when at terminal we enter CNRTL-\
*  SIGILL         4    Core     Generated when we executed an illegal instruction
*  SIGTRAP     5    Core     Trace trap (not reset when caught)
*  SIGABRT     6    Core     Generated by the abort function
*  SIGFPE       8    Core     Floating Point error
*  SIGKILL      9    Termin.  Termination (can't catch, block, ignore)
*  SIGBUS     10    Core     Generated in case of hardware fault
*  SIGSEGV   11    Core     Generated in case of illegal address
*  SIGSYS      12    Core     Generated when we use a bad argument in a system service call
*  SIGPIPE     13    Termin.  Generated when writing to a pipe or a socket while no process is reading at other end
*  SIGALRM   14    Termin.  Generated by clock when alarm expires
*  SIGTERM   15    Termin.  Software termination signal
*  SIGURG     16    Ignore   Urgent condition on IO channel
*  SIGCHLD   20    Ignore   A child process has terminated or stopped
*  SIGTTIN     21    Stop     Generated when a backgorund process reads from terminal
*  SIGTTOUT  22    Stop     Generated when a background process writes to terminal
*  SIGXCPU    24    Discard  CPU time has expired
*  SIGUSR1    30    Termin.  User defiled signal 1
*  SIGUSR2    31    Termin.  User defined signal 2
*
* @author Javier Hidalgo Carrio .
*/

/**
* @brief POINTER TO HANDLING FUNCTION
*
* Defines for the signaling library. Definitiod of data type for pointer to handling function
*/
typedef void (* handling_t) (int sig, void *data);

/**
* @brief GLOBAL ARRAY.
*
*  Two arrays:
*  		handling_t htab: one for pointer of function (the handling function)
* 		data: array of handling function parameters. 
*/

static handling_t htab [20] ;
static void *data[20];

/**
* @brief Function that set a generic handler for a signal.
* 
* @author Javier Hidalgo Carrio.
* @param[in] sig integer for the number of signal to be handled
*
* @return Void
*
*/
static void generic_handling (int sig)
{

   htab[sig](sig,data[sig]);

}

/**
* @brief Function that set a defined handler for a signal.
* 
* @author Javier Hidalgo Carrio.
* @param[in] sig integer for the number of signal to be handled
* @param[in] pfunc pointer of the handling function
* @param[in] values pointer of void, the parameter of the handling function.
*
* @return OK is everything all right. ERROR on other cases
*
*/
bool signal_set (int sig,handling_t pfunc,void *values)
{

	htab[sig] = pfunc;
	data[sig] = values;
	if ((signal (sig,generic_handling))==SIG_ERR)
	 return false;
	return true;
}


/**
* @brief The Function catchs the signal introduced in the parameter of this function
* 
* The function associates the signals with the handling function to be handled when
* the process would receives such signal from the operating system.
* The handling function should have two parameters when defined in the main program.
* one parameter (integer) for the signal number, another for the pointer to void (values)
* Normally the last one is a pointer to a structure.
*
* @author Javier Hidalgo Carrio.
* @param[in] sig_1 integer for the number of signal 1 to be handled
* @param[in] sig_2 integer for the number of signal 2 to be handled
* @param[in] sig_3 integer for the number of signal 3 to be handled
* @param[in] sig_4 integer for the number of signal 4 to be handled
* @param[in] pfunc pointer to the handling function
* @param[in] values pointer to void, the parameter of the handling function.
*
* @return OK is everything all right. ERROR in other cases
*
*/
bool signal_catcher (int sig_1,int sig_2,int sig_3, int sig_4, handling_t pfunc, void *values)
{
	bool status=true;

	status=signal_set (sig_1,pfunc,values);

	if (status)

	status=signal_set (sig_2,pfunc,values);

	if (status)

	status=signal_set (sig_3,pfunc,values);

	if (status)

	status=signal_set (sig_4,pfunc,values);
	
	return status;

}

/**
 * @brief Function for handling system signals.
 * 
 * This function is the handler when the catched
 * system signal happen.
 *
 * @author Javier Hidalgo Carrio.
 *
 * @param[in] sig integer of the catched signal
 * @param[in] values pointer of void (pointer of the IMU structure after casting in the function)
 *
 * @return void
 *
 */
void signal_terminator (int sig, void *values)
{
	bool status = true;
	imar::iVRU_BB * pimar;

	pimar = (imar::iVRU_BB *) values;

	if (sig == SIGINT)
		std::cout<< "\nSIGINT: Terminal interrupt\n";
	else if (sig == SIGTERM)
		std::cout<< "\nSIGTERM: Termination\n";
	else if (sig == SIGHUP)
		std::cout<< "\nSIGHUP: Hangup\n";
	else std::cout<<"\nSIGSEGV: Segmentation\n";


	
	if ((status = pimar->close_port()))
	{
	    std::cout<<"serial port closed correctly.\n";
	}

	
  	exit (status);
	
}



int main(int argc, char** argv)
{
	/****** Variables ******/
	unsigned char values[imar::PKG_SIZE];
	unsigned char sync_word = 0x7E;
	int byteRead;
	imar::iVRU_BB imar;
	handling_t pfunc;
	
	/* Function signal handling */
	pfunc = signal_terminator;
	
	/* Catching system signals */
	signal_catcher (SIGHUP,SIGINT,SIGTERM, SIGSEGV, pfunc, &imar);
	
	imar.welcome();
	
	/** Open serial port **/
	std::cout<<"Connecting to IMU("<<argv[1]<<").......";
	
	byteRead = 0;
	
	
	
	/** Argv[1] normally is "/dev/ttyUSB0" driver description file on Linux system **/
	if (imar.init_serial(argv[1], 57600))
	{
	    std::cout<<"OK\n";
	    
	    if (imar.cbIsEmpty())
		std::cout<<"Buffer is empty\n";
	    
	    while(true)
	    {
		std::cout<<"File Descriptor: "<<imar.getDescriptor()<<"\n";
		std::cout<<"sizeof(values): "<<std::dec<<sizeof(values)<<"\n";
		
		if ((byteRead = imar.read_serial(values, sizeof(values))))
		{
		    std::cout<<"byteRead: "<<byteRead<<"\n";
		    
		    for (int i=0; i< byteRead; i++)
		    {
			if (values[i] == sync_word)
			{
			    std::cout<<"FOUND at ("<<std::dec<<i<<")";
			    printf("%X \n", values[i]);
			}
			else
			{
			    std::cout<<"Values("<<std::dec<<i<<")";
			    printf("%X \n", values[i]);
			}

// 			if (i==0)
// 			{
// 			    std::cout<<"Previous Values("<<std::dec<<i-1<<")";
// 			    printf("%X \n", values[i-1]);
// 			}
			    
		    }
		    
		    /** Store the read values in the Circular Buffer **/
		    imar.cbWritePckg (byteRead, values);
		    std::cout<<"Writen "<<byteRead<<" bytes in Circular Buffer\n";

		    if (imar.cbIsFull())
		    {
			std::cout<<"Circular Buffer full\n";
			
			if (!imar.cbIsSynchronized()) 
			{
			    /**Synchronize with the starting byte**/
			    if (imar.cbSynchronize())
			    {
				std::cout<<"Synchronized!!!\n";
			    }
			}
		    }
		    else if (imar.cbIsSynchronized()) 
		    {
			imar.cbReadValues();
			imar.cbPrintValues();
		    }
		}
		else
		{
		    std::cout<<"ERROR: reading the port\n";
		}
	    
 		usleep (10000);//it is set at 100Hz. 0.01 second which are 10000microseconds
	    }
	    imar.close_port();
	}
	return 0;
}


