/**\file imar.cpp
 *
 * This class has the primitive methods for the IMAR IMU driver 
 * model iVRU-BB. Methods provide the basic for managing the
 * serial port connection and acquiring inertia value in the binary float
 * coding of the values coming from the IMU.
 * 
 * The version 1.0 only allows to acquired data in the default mode
 * whatever this model is in the baud range and specified samplind rate
 * for a package size defined by PKG_SIZE (bynary float data format).
 * It is desired to extend the
 * driver for allowing configuration using
 * the debug serial port communication which allows to change and configure
 * several parameters by writing to this serial port.
 * 
 * @author Javier Hidalgo Carrio | DFKI RIC Bremen | javier.hidalgo_carrio@dfki.de
 * @date August 2012.
 * @version 1.0.
 */


#include <stdio.h>   /**< Standard input/output definitions */
#include <string.h>  /**< String function definitions */
#include <unistd.h>  /**< UNIX standard function definitions */
#include <fcntl.h>   /**< File control definitions */
#include <errno.h>   /**< Error number definitions */
#include <termios.h> /**< POSIX terminal control definitions */
#include <math.h> /** Math includes **/
#include <stdint.h>
#include "imar.hpp"

namespace imar
{
    union IntFloat
    {
      uint32_t i;
      float f;
    };
    
    iVRU_BB::iVRU_BB()
    {
	
	this->fd = 0;
	
	/** Init the circular Buffer **/
	myBuffer.synchronized = false;
	myBuffer.size = sizeof(myBuffer.data);
	myBuffer.write = 0;
	myBuffer.read = 0;

    }
    
    iVRU_BB::~iVRU_BB()
    {

    }

    /**
    * @brief This method opens a serial port
    */
    int iVRU_BB::open_port(const char* name)
    {

	this->fd = open (name, O_RDWR | O_NOCTTY |  O_NDELAY);
	if (this->fd == ERROR)
	{
		/*
		*  Could not open the port.
		*/

		perror("open_port: Unable to open ");
		return ERROR;
	}
	else
	{
		 fcntl(this->fd, F_SETFL, FNDELAY);
		 std::cout<<"IMU Com Port iVRU_BB descriptor is: "<<this->fd<<"\n";
		 
		 return OK;
	}	
	
    }
    
    /**
    * @brief This method gives the baud rate in speed_t termios struct
    */
    speed_t getBaudsRate(const int bauds)
    {
	speed_t speed;
	
	switch(bauds)
	{
	    case 50:
		speed = B50;
		
	    case 75:
		speed = B75;
		
	    case 110:
		speed = B110;
		
	    case 134:
		speed = B134;
		
	    case 150:
		speed = B150;
		
	    case 200:
		speed = B200;
		
	    case 300:
		speed = B300;
		
	    case 600:
		speed = B600;
		
	    case 1200:
		speed = B1200;
		
	    case 1800:
		speed = B1800;
		
	    case 4800:
		speed = B4800;
		
	    case 9600:
		speed = B9600;
		
	    case 19200:
		speed = B19200;
		
	    case 38400:
		speed = B38400;
		
	    case 57600:
		speed = B57600;
		
	    case 115200:
		speed = B115200;
		
	    case 230400:
		speed = B230400;
		
	    default:
		return ERROR;
	}
	
	return speed;
    }
    
    /**
    * @brief This method opens and configures a serial port
    */
    int iVRU_BB::init_serial(const char* name, const int bauds)
    {
	/****** variables ******/
	
	struct termios PortSettings;
	speed_t speed;

	
	if (open_port(name) != ERROR)
	{

		/** Get the current options for the port **/
		tcgetattr(fd,&PortSettings);
		
		speed =  getBaudsRate(bauds);
		
		/** Set the baud rates to 57600 **/
// 		cfsetispeed(&PortSettings, speed); 
 		cfsetospeed(&PortSettings, speed);

		/** Input flags:
		 Disable input processing: no parity checking or marking, no
		 signals from break conditions, do not convert line feeds or
		 carriage returns, and do not map upper case to lower case.
		*/
		PortSettings.c_iflag &= ~(INPCK | PARMRK | BRKINT | INLCR | ICRNL | IUCLC | ICRNL);
		PortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);

		/* ignore break conditions */
		PortSettings.c_iflag |= IGNBRK;
		
		/*Line Options flags*/
		PortSettings.c_lflag &= ~(ICANON | ECHO | ISIG);

		/*Output flags*/
		PortSettings.c_oflag &= ~OPOST; /** no output processing */
		PortSettings.c_oflag &= ~ONLCR; /** don't convert line feeds */

		/*Control Options Flags */
		PortSettings.c_cflag |= (CLOCAL | CREAD);
		
		/*Set 8 bits, 1 stop bit,no flow control*/
		PortSettings.c_cflag &= ~PARENB;
		PortSettings.c_cflag &= ~CSTOPB;
		PortSettings.c_cflag &= ~CSIZE;
		PortSettings.c_cflag |= CS8; /** Select 8 data bits */
		PortSettings.c_cflag |= CRTSCTS;
		
		/*trash the initial IMT30 message */
		tcflush(fd,TCOFLUSH);
		tcflush (fd, TCIFLUSH);

		tcsetattr(fd, TCSANOW, &PortSettings);
		
		return OK;
	}
	
	return ERROR;
    }


    /**
    * @brief This method write values in the serial port
    */
    int iVRU_BB::write_serial(char* command, int nbytes)
    {
	int n = 0;	
	
	if (fd != ERROR)
	{
		//printf ("In write function: %s tamanio: %d", command, nbytes);
		n = write(fd, command, nbytes);
		if (n<0)
		{
			fputs("write()  failed!\n", stderr);
			return ERROR;
		}
		else
		{
			return OK;
		}	
	}
	return ERROR;
    }
    
    /**
    * @brief Function to read values from a serial port connection.
    */
    int iVRU_BB::read_serial(unsigned char* bufferpr, int nbytes)
    {
	
	int n = 0;
// 	std::cout<<"Buffer size: "<<std::dec<<nbytes<<"\n";
	
	if (this->fd != ERROR)
	{
		if ((n=read (this->fd, (unsigned char *) bufferpr, nbytes)) > 0)
		{
// 		    std::cout<<"Read: "<<std::dec <<n<<"\n";
		    return n;
		}
	}
	return ERROR;
    }

    
    /**
    * @brief Function to close an open serial port connection.
    */
    int iVRU_BB::close_port()
    {
	
	if (fd != ERROR)
	{
		return close (fd); /**< Returns OK if no errors*/
		
	}
		return ERROR;
    }

    /**
    * @brief Dummy Welcome methods
    */
    void iVRU_BB::welcome()
    {
	    std::cout << "You successfully compiled and executed iMAR iVRU-BB Driver ("<<this->fd<<")"<< std::endl;
    }
    
    /**
    * @brief Returns file decriptor
    */
    int iVRU_BB::getDescriptor ()
    {
	return this->fd;
    }
    
    /**
    * @brief Returns if the circular buffer is full
    */
    int iVRU_BB::cbIsFull()
    {
	return (myBuffer.write + 1) % myBuffer.size == myBuffer.read;
    }
    
    /**
    * @brief Returns if the circular buffer is Empty
    */
    int iVRU_BB::cbIsEmpty()
    {
	return myBuffer.write == myBuffer.read;
    }
    
    /**
    * @brief Returns if the driver is Synchronized
    */
    int iVRU_BB::cbIsSynchronized()
    {
	return myBuffer.synchronized;
    }
    
    void iVRU_BB::cbWritePckg(int nbytes, unsigned char *values)
    {
	for (int i=0; i< nbytes; ++i)
	{
	   myBuffer.data[myBuffer.write] = values[i];
// 	   std::cout<<"Buffer("<<std::dec<<myBuffer.write<<")";
// 	   printf("%X \n", myBuffer.data[myBuffer.write]);
// 	   std::cout<<"write("<<myBuffer.write<<") read("<<myBuffer.read<<")\n";
	   myBuffer.write = (myBuffer.write + 1) % myBuffer.size;
	   if (myBuffer.write == myBuffer.read)
	    myBuffer.read = (myBuffer.read + 1) % myBuffer.size; /* full, overwrite */
	}
    }
    
    int iVRU_BB::cbSynchronize ()
    {
	int sync = myBuffer.read;
	UINT16 length;
	int ilength;
	
	if (!iVRU_BB::cbIsEmpty())
	{
// 	    std::cout<< "Cb is not empty\n";
	    
	    while (sync != myBuffer.write)
	    {
// 		std::cout<<"Searching("<<std::dec<<sync<<")";
// 		printf("%X \n", myBuffer.data[sync]);
// 		std::cout<< "sync("<<sync<<") write("<<myBuffer.write<<") read("<<myBuffer.read<<")\n";
		
		sync = (sync + 1) % myBuffer.size;
			    
		if (myBuffer.data[sync] == SYNC_WORD)
		{
// 		    std::cout<<"FOUND at ("<<std::dec<<sync<<")";
// 		    printf("%X \n", myBuffer.data[sync]);
		    
// 		    std::cout<<"Counter("<<std::dec<<(sync+1)% myBuffer.size<<")";
// 		    printf("%X \n", myBuffer.data[(sync+1)% myBuffer.size]);
		    
		    myBuffer.counter = myBuffer.data[(sync+1)% myBuffer.size];
// 		    std::cout<<"Counter+1: ";
// 		    printf("%X \n",  myBuffer.counter+1);
		    
// 		    std::cout<<"Next Synchronize at ("<<std::dec<<(sync+PKG_SIZE)%myBuffer.size<<")";
// 		    printf("%X \n", myBuffer.data[(sync+PKG_SIZE)%myBuffer.size]);
		    
// 		    std::cout<<"Counter+1("<<std::dec<<(sync+PKG_SIZE+1)%myBuffer.size<<")";
// 		    printf("%X \n", myBuffer.data[(sync+PKG_SIZE+1)%myBuffer.size]);
// 		    printf("It should be: %X \n", myBuffer.counter + 1);
		    
		    if ((myBuffer.data[(sync+PKG_SIZE+1)%myBuffer.size]) == myBuffer.counter + 1)
		    {
			myBuffer.synchronized = true;
			
			/** Jump one package ahead to be fully sync**/
			/** The app must read new values again **/
			myBuffer.read = (sync+PKG_SIZE)%myBuffer.size;
			
			length.data[1] = myBuffer.data[(sync+2)%myBuffer.size];
			length.data[0] = myBuffer.data[(sync+3)%myBuffer.size];
			ilength = (length.data[0] << 8)|(0x00FF & length.data[1]);
			
// 			std::cout<< "Length: "<< ilength<<"\n";
// 			printf("%X %X \n", length.data[0], length.data[1]);
			break;
		    }
		}
	    }
	}
	
	if (myBuffer.synchronized == true)
	{
	    return OK;
	}
	else
	{	
	    return ERROR;
	}
    }
    
    
    int iVRU_BB::cbNumberElements()
    {
	
	if (myBuffer.read <= myBuffer.write)
	{
	    return (myBuffer.write - myBuffer.read);
	}
	else 
	{   
	    /** From read till the end **/
	    int part1 = sizeof(myBuffer.data) - myBuffer.read;
	    
	    return part1 + myBuffer.write - 1;
	}
	    

    }

    int iVRU_BB::cbReadPckg(unsigned char *pckg)
    {
	register int i;
	
	if (pckg == NULL)
	    return ERROR;
	else
	{
	    if ((sizeof(pckg) < PKG_SIZE) || (myBuffer.synchronized == false))
		return ERROR;
	}
	
	for (i=0; i<PKG_SIZE; ++i)
	{
	    pckg[i] = myBuffer.data[myBuffer.read];
	    
// 	    std::cout<<"Pckg("<<i<<")";
// 	    printf("%X \n", pckg[i]);
	    
	    if (!iVRU_BB::cbIsEmpty())
		myBuffer.read = (myBuffer.read + 1) % myBuffer.size;
	    else
		break;
	}
	
	if (i == PKG_SIZE)
	    return OK;
	else
	    return ERROR;
    }
    
    int iVRU_BB::cbReadValues()
    {
	unsigned char number[4];
	union IntFloat val;
	UINT16 length;
	
	if (iVRU_BB::cbIsSynchronized())
	{
	    
// 	    printf("SyncWord (%d): %X\n", (myBuffer.read)%myBuffer.size, myBuffer.data[(myBuffer.read)%myBuffer.size]);
	    
	    /** Pckg counter **/
	    myIMU.counter = myBuffer.data[(myBuffer.read+1)%myBuffer.size];
	    
	    /** Pckg length **/
	    length.data[1] = myBuffer.data[(myBuffer.read+2)%myBuffer.size];
	    length.data[0] = myBuffer.data[(myBuffer.read+3)%myBuffer.size];
	    myIMU.length = (length.data[0] << 8)|(0x00FF & length.data[1]);
	    
// 	    std::cout<< "DataPckg Length: "<< myIMU.length<<"\n";
	    
	    
	    /** Accelerometers X **/
	    number[0] = myBuffer.data[(myBuffer.read+5)%myBuffer.size];
	    number[1] = myBuffer.data[(myBuffer.read+4)%myBuffer.size];
	    
	    number[2] = myBuffer.data[(myBuffer.read+7)%myBuffer.size];
	    number[3] = myBuffer.data[(myBuffer.read+6)%myBuffer.size];
	    
	    val.i = (number[0] << 24 )|(0x00FF0000 & number[1] << 16)|(0x0000FF00 & number[2] << 8)|(0x000000FF & number[3]);
	    myIMU.acc[0] = val.f;
	    
	    /** Accelerometers Y **/
	    number[0] = myBuffer.data[(myBuffer.read+9)%myBuffer.size];
	    number[1] = myBuffer.data[(myBuffer.read+8)%myBuffer.size];
	    
	    number[2] = myBuffer.data[(myBuffer.read+11)%myBuffer.size];
	    number[3] = myBuffer.data[(myBuffer.read+10)%myBuffer.size];
	    
	    val.i = (number[0] << 24 )|(0x00FF0000 & number[1] << 16)|(0x0000FF00 & number[2] << 8)|(0x000000FF & number[3]);
	    myIMU.acc[1] = val.f;
	    
	    /** Accelerometers Z **/
	    number[0] = myBuffer.data[(myBuffer.read+13)%myBuffer.size];
	    number[1] = myBuffer.data[(myBuffer.read+12)%myBuffer.size];
	    
	    number[2] = myBuffer.data[(myBuffer.read+15)%myBuffer.size];
	    number[3] = myBuffer.data[(myBuffer.read+14)%myBuffer.size];
	    
	    val.i = (number[0] << 24 )|(0x00FF0000 & number[1] << 16)|(0x0000FF00 & number[2] << 8)|(0x000000FF & number[3]);
	    myIMU.acc[2] = val.f;
	    
	    /** Gyroscope X **/
	    number[0] = myBuffer.data[(myBuffer.read+17)%myBuffer.size];
	    number[1] = myBuffer.data[(myBuffer.read+16)%myBuffer.size];
	    
	    number[2] = myBuffer.data[(myBuffer.read+19)%myBuffer.size];
	    number[3] = myBuffer.data[(myBuffer.read+18)%myBuffer.size];
	    
	    val.i = (number[0] << 24 )|(0x00FF0000 & number[1] << 16)|(0x0000FF00 & number[2] << 8)|(0x000000FF & number[3]);
	    myIMU.gyro[0] = D2R * val.f;
	    
	    /** Gyroscope Y **/
	    number[0] = myBuffer.data[(myBuffer.read+21)%myBuffer.size];
	    number[1] = myBuffer.data[(myBuffer.read+20)%myBuffer.size];
	    
	    number[2] = myBuffer.data[(myBuffer.read+23)%myBuffer.size];
	    number[3] = myBuffer.data[(myBuffer.read+22)%myBuffer.size];
	    
	    val.i = (number[0] << 24 )|(0x00FF0000 & number[1] << 16)|(0x0000FF00 & number[2] << 8)|(0x000000FF & number[3]);
	    myIMU.gyro[1] = D2R * val.f;
	    
	    /** Gyroscope Z **/
	    number[0] = myBuffer.data[(myBuffer.read+25)%myBuffer.size];
	    number[1] = myBuffer.data[(myBuffer.read+24)%myBuffer.size];
	    
	    number[2] = myBuffer.data[(myBuffer.read+27)%myBuffer.size];
	    number[3] = myBuffer.data[(myBuffer.read+26)%myBuffer.size];
	    
	    val.i = (number[0] << 24 )|(0x00FF0000 & number[1] << 16)|(0x0000FF00 & number[2] << 8)|(0x000000FF & number[3]);
	    myIMU.gyro[2] = D2R * val.f;
	    
	    /** Roll **/
	    number[0] = myBuffer.data[(myBuffer.read+29)%myBuffer.size];
	    number[1] = myBuffer.data[(myBuffer.read+28)%myBuffer.size];
	    
	    number[2] = myBuffer.data[(myBuffer.read+31)%myBuffer.size];
	    number[3] = myBuffer.data[(myBuffer.read+30)%myBuffer.size];
	    
	    val.i = (number[0] << 24 )|(0x00FF0000 & number[1] << 16)|(0x0000FF00 & number[2] << 8)|(0x000000FF & number[3]);
	    myIMU.euler[0] = D2R * val.f;
	    
	    
	    /** Pitch **/
	    number[0] = myBuffer.data[(myBuffer.read+33)%myBuffer.size];
	    number[1] = myBuffer.data[(myBuffer.read+32)%myBuffer.size];
	    
	    number[2] = myBuffer.data[(myBuffer.read+35)%myBuffer.size];
	    number[3] = myBuffer.data[(myBuffer.read+34)%myBuffer.size];
	    
	    val.i = (number[0] << 24 )|(0x00FF0000 & number[1] << 16)|(0x0000FF00 & number[2] << 8)|(0x000000FF & number[3]);
	    myIMU.euler[1] = D2R * val.f;
	    
	    /** Yaw **/
	    number[0] = myBuffer.data[(myBuffer.read+37)%myBuffer.size];
	    number[1] = myBuffer.data[(myBuffer.read+36)%myBuffer.size];
	    
	    number[2] = myBuffer.data[(myBuffer.read+39)%myBuffer.size];
	    number[3] = myBuffer.data[(myBuffer.read+38)%myBuffer.size];
	    
	    val.i = (number[0] << 24 )|(0x00FF0000 & number[1] << 16)|(0x0000FF00 & number[2] << 8)|(0x000000FF & number[3]);
	    myIMU.euler[2] = D2R * val.f;
	    
// 	    val.i = 0x3f9d70a4;
// 	    val.i = (0x3f << 24 )|(0x00FF0000 & 0x9d << 16)|(0x0000FF00 & 0x70 << 8)|(0x000000FF & 0xa4);
// 	    printf("Union %f\n", val.f);
	    	    
	    myBuffer.read = (myBuffer.read+PKG_SIZE)%myBuffer.size;
// 	    printf("Next->SyncWord (%d): %X\n", (myBuffer.read)%myBuffer.size, myBuffer.data[(myBuffer.read)%myBuffer.size]);
	    
	    
	    return OK;
	}
	else
	{
	    return ERROR;
	}
    }
    
    void iVRU_BB::cbPrintValues()
    {
	/** Inertial Values **/
	std::cout<<"AccX: "<<myIMU.acc[0]<<"AccY: "<<myIMU.acc[1]<<"AccZ: "<<myIMU.acc[2]<<"\n";
	std::cout<<"GyroX: "<<myIMU.gyro[0]<<"GyroY: "<<myIMU.gyro[1]<<"GyroZ: "<<myIMU.gyro[2]<<"\n";
	std::cout<<"Roll: "<<myIMU.euler[0]<<"Pitch: "<<myIMU.euler[1]<<"Yaw: "<<myIMU.euler[2]<<"\n";
    }
    
    Eigen::Matrix< double, NUMAXIS , 1  > iVRU_BB::getAccelerometers()
    {
	Eigen::Matrix< double, NUMAXIS , 1  > acc;
	
	acc[0] = myIMU.acc[0];
	acc[1] = myIMU.acc[1];
	acc[2] = myIMU.acc[2];
	
	return acc;

    }

    Eigen::Matrix< double, NUMAXIS , 1  > iVRU_BB::getGyroscopes()
    {
	Eigen::Matrix< double, NUMAXIS , 1  > gyro;
	
	gyro[0] = myIMU.gyro[0];
	gyro[1] = myIMU.gyro[1];
	gyro[2] = myIMU.gyro[2];
	
	return gyro;

    }

    Eigen::Quaternion< double > iVRU_BB::getAttitude()
    {
	Eigen::Quaternion< double > attitude;
	
	attitude = Eigen::Quaternion <double> (Eigen::AngleAxisd(myIMU.euler[2], Eigen::Vector3d::UnitZ())*
 			    Eigen::AngleAxisd(myIMU.euler[1], Eigen::Vector3d::UnitY()) *
 			    Eigen::AngleAxisd(myIMU.euler[0], Eigen::Vector3d::UnitX()));
	
	return attitude;
    }
    
    int iVRU_BB::getPacketCounter()
    {
	return myIMU.counter;

    }


    
}