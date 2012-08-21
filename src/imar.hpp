/**\file imar.hpp
 * Header function file and defines
 */

#ifndef _IMAR_HPP_
#define _IMAR_HPP_

#include <iostream>
#include <Eigen/Geometry> /**< Eigen data type for Matrix, Quaternion, etc... */


namespace imar
{
    
    /** General defines **/
    #ifndef OK
    #define OK	0 /** Integer value in order to return when everything is all right. */
    #endif
    
    #ifndef ERROR
    #define ERROR -1 /** Integer value in order to return when an error occur. */
    #endif
    
    #ifndef PKG_SIZE
    #define PKG_SIZE 50 /** Size of the package coming from the IMU **/
    #endif
    
    #ifndef SYNC_WORD
    #define SYNC_WORD 0x7E /** Synchronization byte of the package (start of a package) **/
    #endif
    
    #ifndef NUMAXIS
    #define NUMAXIS 3 /** NUmber of axis of the IMU **/
    #endif
    
    #ifndef PI
    #define PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286 /**< Pi Number */
    #endif
  
    #ifndef D2R
    #define D2R PI/180.00 /** Convert degree to radian **/
    #endif
    
    #ifndef R2D
    #define R2D 180.00/PI /** Convert radian to degree **/
    #endif
	
    typedef struct {
	unsigned char data[2];
    }UINT16; /** 16 bits */

    /** Circular buffer object **/
    typedef struct {
	bool synchronized; /** true is synchronized **/
	unsigned char counter; /** package counter **/
	int size;   /** maximum number of elements*/
	int read;  /** index of read element */
	int write;    /** index at which to write new element*/
	unsigned char data[(2*PKG_SIZE) + 1];  /* vector of elements */
    } CircularBuffer;
    
    /** Raw IMU data **/
    typedef struct {
	unsigned char counter; /** package counter **/
	int length; /** Number of the information bytes **/
// 	unsigned char ibit[8]; /** Init Build in Test (IBIT) information check info **/
	float acc[NUMAXIS]; /** Accelerometers values **/
	float gyro[NUMAXIS]; /** Gyros raw values **/
	float euler[NUMAXIS]; /** Roll, Pitch and Yaw values **/
	UINT16 crc; /** CRC-CCITT **/
    } ImuData;
    


    class iVRU_BB
    {
	
	
	
	private:
	    int fd;
	    CircularBuffer myBuffer;
	    ImuData myIMU;
	    
	public: 
	    
	    /**
	    * Print a welcome to stdout
	    * \return nothing
	    */
	    iVRU_BB();
	    
	    ~iVRU_BB();
	    
	    /**
	    * @brief Dummy Welcome method
	    * 
	    * @author Javier Hidalgo Carrio.
	    *
	    * @return void
	    *
	    */
	    void welcome();
	    
	    /**
	    * @brief This method opens a serial port
	    * 
	    * The method opens a serial port with descriptor file
	    * name specified in the paramaters.
	    *
	    * @author Javier Hidalgo Carrio.
	    *
	    * @param[in] *name pointer to char (C type string) with the descriptor file name
	    *
	    * @return OK is everything all right. ERROR on other cases.
	    *
	    */
	    int open_port (const char* name);

	    /**
	    * @brief This method opens and configures a serial port
	    * 
	    * The method opens a serial port with descriptor file
	    * name and the bauds rate specified.
	    * The serial port is open set to no parity cheking
	    * 8databits, 1 stopbit and no hardware protocol.
	    *
	    * @author Javier Hidalgo Carrio.
	    *
	    * @param[in] *name pointer to char (C type string) with the descriptor file name
	    * @param[out] bauds integer value with the baud rate
	    *
	    * @return OK is everything all right. ERROR on other cases.
	    *
	    */
	    int init_serial (const char *name, const int bauds);
	    
	    
	    /**
	    * @brief This method write values in the serial port
	    * 
	    * Write values in the serial port
	    *
	    * @author Javier Hidalgo Carrio.
	    *
	    * @param[in] *commad pointer to array of values
	    * @param[out] nbytes number of byte to write
	    *
	    * @return OK is everything all right. ERROR on other cases.
	    *
	    */
	    int write_serial(char* command, int nbytes);
	    
	    
	    /**
	    * @brief Function to read values from a serial port connection.
	    * 
	    * This function read nbytes from the open serial port
	    *
	    * @author Javier Hidalgo Carrio.
	    *
	    * @param[out] bufferpr pointer to char. Buffer to store the values read
	    * @param[in] nbytes number of bytes to read.
	    *
	    * @return OK if everything all right, ERROR on error.
	    *
	    */
	    int read_serial(unsigned char * bufferpr, int nbytes);

	    /**
	    * @brief Function to close an open serial port connection.
	    * 
	    * This function closes the serial port connection.
	    *
	    * @author Javier Hidalgo Carrio.
	    *
	    * @return OK if everything all right, ERROR on error.
	    *
	    */
	    int close_port ();
	    
	    /**
	    * @brief Returns file decriptor
	    *
	    * @author Javier Hidalgo Carrio.
	    *
	    * @return the fd value
	    *
	    */
	    int getDescriptor();
	    
	    
	    /**
	    * @brief Returns if the circular buffer is full
	    * 	    
	    *
	    * @author Javier Hidalgo Carrio.
	    *
	    * @return true if buffer is full false in other cases.
	    *
	    */
	    int cbIsFull();
	    
	    /**
	    * @brief Returns if the circular buffer is Empty
	    * 	    
	    *
	    * @author Javier Hidalgo Carrio.
	    *
	    * @return true if buffer is Empty false in other cases.
	    *
	    */
	    int cbIsEmpty();
	    
	    /**
	    * @brief Returns if the driver is Synchronized
	    * 
	    * When the read pointer of the cicular buffer is
	    * synchronized with the SyncWord of the stream the
	    * driver is synchronized.	    
	    *
	    * @author Javier Hidalgo Carrio.
	    *
	    * @return true if it is Synchronized false in other cases.
	    *
	    */
	    int cbIsSynchronized();
	    
	    
	    /**
	    * @brief It writes information in the Cicular Buffer
	    * 
	    * The values previously read in the serial port are writen
	    * in the Circular Buffer.
	    *
	    * @author Javier Hidalgo Carrio.
	    *
	    * @param[in] nbytes number of bytes to read.
	    * @param[in] values pointer to char. Buffer to store the values read
	    * 
	    * @return void
	    *
	    */
	    void cbWritePckg(int nbytes, unsigned char * values);
	    
	    /**
	    * @brief It synchronizes the circular buffer
	    * 
	    * The method synchronizes the cicular buffer
	    * seeking for the Synchronization Word defined by SYNC_WORD
	    *
	    * @author Javier Hidalgo Carrio.
	    *
	    * 
	    * @return OK if synchronization makes success. ERROR in other cases.
	    *
	    */
	    int cbSynchronize ();
	    
	    /**
	    * @brief It reads a information from the circular buffer
	    * 
	    * The method reads raw information from the circular buffer.
	    *
	    * @author Javier Hidalgo Carrio.
	    *
	    * @param[out] *pckg pointer to buffer.
	    * 
	    * @return OK if everything is good, ERROR in other cases.
	    *
	    */
	    int cbReadPckg(unsigned char *pckg);
	    
	    /**
	    * @brief It reads a IMU packages
	    * 
	    * The method reads the information in the circular buffer
	    * and gives the inertial information. It needs to be
	    * previously synchronized otherwise the values would be wrong.
	    * The values are stored in the ImuData variable.
	    *
	    * @author Javier Hidalgo Carrio.
	    *
	    * @return OK if everything is good, ERROR in other cases.
	    *
	    */
	    int cbReadValues();
	    
	    /**
	    * @brief It prints the values stored in ImuData
	    *
	    * @author Javier Hidalgo Carrio.
	    *
	    * @return OK if everything is good, ERROR in other cases.
	    *
	    */
	    void cbPrintValues();
	    
	    /**
	    * @brief It gives the number of elements stored in the Circular Buffer
	    *
	    * @author Javier Hidalgo Carrio.
	    *
	    * @return Return the number of elements.
	    *
	    */
	    int cbNumberElements();
	    
	    /**
	    * @brief It gives the package counter
	    *
	    * @author Javier Hidalgo Carrio.
	    *
	    * @return Return the incremental counter
	    *
	    */
	    int getPacketCounter();
	    
	    /**
	    * @brief It gets the Accelerometers values.
	    *
	    * @author Javier Hidalgo Carrio.
	    *
	    * @return The Accelerometers.
	    *
	    */
	    Eigen::Matrix <double,NUMAXIS,1> getAccelerometers();
	    
	    /**
	    * @brief It gets the Gyroscopes values.
	    *
	    * @author Javier Hidalgo Carrio.
	    *
	    * @return The Gyroscopes
	    *
	    */
	    Eigen::Matrix <double,NUMAXIS,1> getGyroscopes();
	    
	    
	    /**
	    * @brief It gets the attitude or orientation.
	    *
	    * @author Javier Hidalgo Carrio.
	    *
	    * @return The attitude in a Quaternion form.
	    *
	    */
	    Eigen::Quaternion <double> getAttitude();
	    
	    
	    
    };

} // end namespace 

#endif // _IMAR_HPP_
