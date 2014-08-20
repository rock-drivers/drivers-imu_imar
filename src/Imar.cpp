/**\file Imar.cpp
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
#include "Imar.hpp"

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif

#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif


namespace imu_imar
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
	
	cbAccX.set_capacity(imu_imar::FFT_WINDOWS_SIZE);

    }
    
    iVRU_BB::~iVRU_BB()
    {

    }

    /**
    * @brief This method opens a serial port
    */
    bool iVRU_BB::open_port(const char* name)
    {

	this->fd = open (name, O_RDWR | O_NOCTTY |  O_NDELAY);
	if (this->fd == -1)
	{
		/*
		*  Could not open the port.
		*/

		perror("open_port: Unable to open ");
		return false;
	}
	else
	{
		 fcntl(this->fd, F_SETFL, FNDELAY);
		 std::cout<<"IMU Com Port iVRU_BB descriptor is: "<<this->fd<<"\n";
		 
		 return true;
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
		break;
		
	    case 75:
		speed = B75;
		break;
		
	    case 110:
		speed = B110;
		break;
		
	    case 134:
		speed = B134;
		break;
		
	    case 150:
		speed = B150;
		break;
		
	    case 200:
		speed = B200;
		break;
		
	    case 300:
		speed = B300;
		break;
		
	    case 600:
		speed = B600;
		break;
		
	    case 1200:
		speed = B1200;
		break;
		
	    case 1800:
		speed = B1800;
		break;
		
	    case 4800:
		speed = B4800;
		break;
		
	    case 9600:
		speed = B9600;
		break;
		
	    case 19200:
		speed = B19200;
		break;
		
	    case 38400:
		speed = B38400;
		break;
		
	    case 57600:
		speed = B57600;
		break;
		
	    case 115200:
		speed = B115200;
		break;
		
	    case 230400:
		speed = B230400;
		break;
		
	    default:
		return false;
	}
	
	return speed;
    }
    
    /**
    * @brief This method opens and configures a serial port
    */
    bool iVRU_BB::init_serial(const char* name, const int bauds)
    {
	/****** variables ******/
	
	struct termios PortSettings;
	speed_t speed;

	
	if (open_port(name))
	{

		/** Get the current options for the port **/
		tcgetattr(fd,&PortSettings);
		
// 		std::cout<<"bauds is: "<<bauds<<"\n";
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
		
		return true;
	}
	
	return false;
    }


    /**
    * @brief This method write values in the serial port
    */
    bool iVRU_BB::write_serial(char* command, int nbytes)
    {
	int n = 0;	
	
	if (fd != -1)
	{
		//printf ("In write function: %s tamanio: %d", command, nbytes);
		n = write(fd, command, nbytes);
		if (n<0)
		{
			fputs("write()  failed!\n", stderr);
			return false;
		}
		else
		{
			return true;
		}	
	}
	return false;
    }
    
    /**
    * @brief Function to read values from a serial port connection.
    */
    bool iVRU_BB::read_serial(unsigned char* bufferpr, int nbytes)
    {
	
	int n = 0;
// 	std::cout<<"Buffer size: "<<std::dec<<nbytes<<"\n";
	
	if (this->fd != -1)
	{
		if ((n=read (this->fd, (unsigned char *) bufferpr, nbytes)) > 0)
		{
//  		    std::cout<<"Read: "<<std::dec <<n<<"\n";
		    return n;
		}
	}
	return false;
    }

    
    /**
    * @brief Function to close an open serial port connection.
    */
    bool iVRU_BB::close_port()
    {
	
	if (fd != -1)
	{
		return close (fd); /** Returns OK if no errors*/
		
	}
		return false;
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
    bool iVRU_BB::cbIsSynchronized()
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
    
    bool iVRU_BB::cbSynchronize ()
    {
	int sync = myBuffer.read;
	//UINT16 length;
	
	if (!iVRU_BB::cbIsEmpty())
	{
// 	    std::cout<< "Cb is not empty\n";
	    
	    while (sync != myBuffer.write)
	    {
// 		std::cout<<"Searching("<<std::dec<<sync<<")";
// 		printf("%X \n", myBuffer.data[sync]);
// 		std::cout<< "sync("<<sync<<") write("<<myBuffer.write<<") read("<<myBuffer.read<<")\n";
		
		sync = (sync + 1) % myBuffer.size;
			
		if (myBuffer.data[sync] == imu_imar::SYNC_WORD)
		{
/*		    std::cout<<"FOUND at ("<<std::dec<<sync<<")";
		    printf("%X \n", myBuffer.data[sync]);
		    
		    std::cout<<"Counter("<<std::dec<<(sync+1)% myBuffer.size<<")";
		    printf("%X \n", myBuffer.data[(sync+1)% myBuffer.size]);*/
		    
		    myBuffer.counter = myBuffer.data[(sync+1)% myBuffer.size];
/*		    std::cout<<"Counter+1: ";
		    printf("%X \n",  myBuffer.counter+1);
		    
		    std::cout<<"Next Synchronize at ("<<std::dec<<(sync+imu_imar::PKG_SIZE)%myBuffer.size<<")";
		    printf("%X \n", myBuffer.data[(sync+imu_imar::PKG_SIZE)%myBuffer.size]);
		    
		    std::cout<<"Counter+1("<<std::dec<<(sync+imu_imar::PKG_SIZE+1)%myBuffer.size<<")";
		    printf("%X \n", myBuffer.data[(sync+imu_imar::PKG_SIZE+1)%myBuffer.size]);
		    printf("It should be: %X \n", myBuffer.counter + 1);*/
		    
		    if ((myBuffer.data[(sync+imu_imar::PKG_SIZE+1)%myBuffer.size]) == myBuffer.counter + 1)
		    {
			myBuffer.synchronized = true;
			
			/** Jump one package ahead to be fully sync**/
			/** The app must read new values again **/
			myBuffer.read = (sync+imu_imar::PKG_SIZE)%myBuffer.size;
			
			//length.data[1] = myBuffer.data[(sync+2)%myBuffer.size];
			//length.data[0] = myBuffer.data[(sync+3)%myBuffer.size];

	                //int ilength = (length.data[0] << 8)|(0x00FF & length.data[1]);
// 			std::cout<< "Length: "<< ilength<<"\n";
// 			printf("%X %X \n", length.data[0], length.data[1]);
			break;
		    }
		}
	    }
	}
	
	if (myBuffer.synchronized == true)
	{
	    return true;
	}
	else
	{	
	    return false;
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

    bool iVRU_BB::cbCopyPckg(unsigned char *pckg, int len)
    {
	register int i;
	int read;
	
	if (pckg == NULL)
	    return false;
	else
	{
	    if ((myBuffer.synchronized == false))
		return false;
	}
	
	read = myBuffer.read;
	
	for (i=0; i<len; ++i)
	{
	    pckg[i] = myBuffer.data[read];
	    
//   	    std::cout<<"Pckg("<<i<<")";
//   	    printf("%X \n", pckg[i]);
	    
	    
	    read = (read + 1) % myBuffer.size;
	    
	}
// 	pckg[i-1] = 0x00;
// 	pckg[i-2] = 0x00;
//  	std::cout<<"END Pckg("<<(read) % myBuffer.size<<")"<<"i="<<i<<"\n";
//   	printf("%X \n", myBuffer.data[(read) % myBuffer.size]);
	if (i == len)
	    return true;
	else
	    return false;
    }
    
    bool iVRU_BB::cbReadValues()
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
	    
	    
	    /** Read the time field **/
	    number[0] = myBuffer.data[(myBuffer.read+45)%myBuffer.size];
	    number[1] = myBuffer.data[(myBuffer.read+44)%myBuffer.size];
	    
	    number[2] = myBuffer.data[(myBuffer.read+47)%myBuffer.size];
	    number[3] = myBuffer.data[(myBuffer.read+46)%myBuffer.size];
	    
// 	    printf("%X %X %X %X\n", number[0], number[1], number[2], number[3]);
	    
	    val.i = (number[0] << 24 )|(0x00FF0000 & number[1] << 16)|(0x0000FF00 & number[2] << 8)|(0x000000FF & number[3]);
	    myIMU.microseconds = val.i;
	    
	    
	    /** Push the accelerometers readings to the FFT buffer **/
	    cbAccX.push_back(myIMU.acc[0]);
	    cbAccY.push_back(myIMU.acc[1]);
	    cbAccZ.push_back(myIMU.acc[2]);


	    
// 	    val.i = 0x3f9d70a4;
// 	    val.i = (0x3f << 24 )|(0x00FF0000 & 0x9d << 16)|(0x0000FF00 & 0x70 << 8)|(0x000000FF & 0xa4);
// 	    printf("Union %f\n", val.f);
	    	    
	    myBuffer.read = (myBuffer.read+imu_imar::PKG_SIZE)%myBuffer.size;
// 	    printf("Next->SyncWord (%d): %X\n", (myBuffer.read)%myBuffer.size, myBuffer.data[(myBuffer.read)%myBuffer.size]);
	    
	    /** Check if the buffer is Synchronize **/
	    if (myBuffer.data[myBuffer.read] !=  imu_imar::SYNC_WORD)
		myBuffer.synchronized = false;
	    
	  
	    
	    return true;
	}
	else
	{
	    return false;
	}
    }

    void iVRU_BB::cbPrintValues()
    {
	/** Inertial Values **/
	std::cout<<"Time: "<<myIMU.microseconds<<"\n";
	printf("Counter: %X\n", myIMU.counter);
	std::cout<<"Length: "<<myIMU.length<<"\n";
	std::cout<<"AccX: "<<myIMU.acc[0]<<" AccY: "<<myIMU.acc[1]<<" AccZ: "<<myIMU.acc[2]<<"\n";
	std::cout<<"GyroX: "<<myIMU.gyro[0]<<" GyroY: "<<myIMU.gyro[1]<<" GyroZ: "<<myIMU.gyro[2]<<"\n";
	std::cout<<"Roll: "<<myIMU.euler[0]<<" Pitch: "<<myIMU.euler[1]<<" Yaw: "<<myIMU.euler[2]<<"\n";
    }
    
    Eigen::Matrix< double, imu_imar::NUMAXIS , 1  > iVRU_BB::getAccelerometers()
    {
	Eigen::Matrix< double, imu_imar::NUMAXIS , 1  > acc;
	
	acc[0] = myIMU.acc[0];
	acc[1] = myIMU.acc[1];
	acc[2] = myIMU.acc[2];
	
	return acc;

    }

    Eigen::Matrix< double, imu_imar::NUMAXIS , 1  > iVRU_BB::getGyroscopes()
    {
	Eigen::Matrix< double, imu_imar::NUMAXIS , 1  > gyro;
	
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
    
    void iVRU_BB::cbCalculateAccIntegration(const float omega)
    {
	Eigen::FFT<float> fft;
	std::vector<float> timevec;
	std::vector<float> timevecVel;
	std::vector<float> timevecPos;
	std::vector<std::complex<float> > freqvecVel;
	std::vector<std::complex<float> > freqvecPos;
	
	timevec.resize(imu_imar::FFT_WINDOWS_SIZE);
	timevecVel.resize(imu_imar::FFT_WINDOWS_SIZE);
	timevecPos.resize(imu_imar::FFT_WINDOWS_SIZE);
	freqvecPos.resize(imu_imar::FFT_WINDOWS_SIZE);
	freqvecVel.resize(imu_imar::FFT_WINDOWS_SIZE);
	
// 	std::cout<<"cbAccX.size(): "<<cbAccX.size()<<"\n";
	
	if (cbAccX.size() == imu_imar::FFT_WINDOWS_SIZE)
	{
	    /** Get the Acceleration **/
	    for (int i=0; i<imu_imar::FFT_WINDOWS_SIZE; ++i)
	    {
		timevec[i] = cbAccX[i];
// 		std::cout<<"Acceleration["<<i<<"]: "<<timevec[i]<<"\n";
	    }
	    
	    /** Remove the first elements from the buffer **/
	    cbAccX.pop_front();
	    
	    fft.SetFlag(fft.Unscaled);
	    
	    /** Compute the FFT **/	
	    fft.fwd(freqvecVel,timevec);
	    fft.fwd(freqvecPos,timevec);
	    
	
	    for (int i=0; i<imu_imar::FFT_WINDOWS_SIZE; ++i)
	    {
// 		std::cout<<"Freq["<<i<<"]: "<<freqvecVel[i]<<"\n";
		
		/** Convert Acc to Displacement **/
		freqvecPos[i] = freqvecPos[i] / (std::complex<float>) pow(omega,2);
	    
		/** Convert Acc to Velocity **/
		freqvecVel[i] = freqvecVel[i] / omega;
	    }
	    
	    /** Calculate the Inverse FFT **/
 	    fft.inv(timevecPos,freqvecPos);
 	    fft.inv(timevecVel,freqvecVel);
// 	    
	    
	    /** Fill the Vel and Displacement **/
 	    velocity[0] = timevecVel[0];
   	    displacement[0] = timevecPos[0];
	    
	    std::cout<<"Velocity: "<<timevecVel[0]<<"\n";
	    std::cout<<"Position: "<<timevecPos[0]<<"\n";
	}
	
// 	std::cout<<"ENDS\n";
	
	return;
    }
    
    Eigen::Matrix< double, 3 , 1  > iVRU_BB::getPosition()
    {
	return displacement;
    }
    
    Eigen::Matrix< double, 3 , 1  > iVRU_BB::getVelocity()
    {
	return velocity;

    }
    
    
    unsigned short iVRU_BB::crc16(unsigned char *data_p, int length)
    {
	unsigned int crc = -1;

	// A popular variant complements rem here
	for(int i=0; i<length; ++i)
	{
	    crc = crc ^ data_p[i];
	    
	    for(int j=0;j<8; ++j)
	    {
		// Assuming 8 bits per byte
		if (crc & 0x0001)
		{
		    // if rightmost (least significant) bit is set
		    crc = (crc >> 1) ^ imu_imar::POLY;
		    
		}
		else
		{
		    crc = crc >> 1;
		}
	    }
	}

	return (crc);
    }
    
    
    static unsigned int CrcByte0(unsigned int const Crc, unsigned char const Byte)
    {
	static unsigned int const aCrcTable[256]= {
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
	0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
	0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
	0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
	0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
	0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
	0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
	0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
	0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
	0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
	0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
	0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
	0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
	0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
	0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
	0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
	0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
	0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
	0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
	0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
	0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
	0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
	0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
	0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
	0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
	0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
	0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
	0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
	0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
	0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
	0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 };
	
// 	printf("index: %d\n", (unsigned char) (Crc >> 8) ^ Byte);
// 	printf ("table: %X\n", aCrcTable[(unsigned char)(Crc >> 8) ^ Byte]);
// 	printf ("return: %X\n", (unsigned int)(Crc << 8) ^ aCrcTable[(unsigned char)(Crc >> 8) ^ Byte]);

	return (Crc << 8) ^ aCrcTable[(unsigned char)(Crc >> 8) ^ Byte];
    }

    unsigned int iVRU_BB::Crc16_0(unsigned char const * pByte, unsigned Size)
    {
	unsigned int Crc = 0x0000;
	
	
	while ( Size-- > 0U )
	{
// 	    printf("*pByte++ %X\n",*pByte++);
	    Crc = CrcByte0(Crc, *pByte++);
	}
	
	return Crc;
    }
    
    void iVRU_BB::cbReset()
    {
	myBuffer.write = 0;
	myBuffer.read = 0;
    }




    
}
