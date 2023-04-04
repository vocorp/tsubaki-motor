#include <stdio.h>
#include <unistd.h>
#include <modbus.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <asm/ioctls.h>
#include <iostream>

class Modbus{
	private:
		int ret;
		const int read_nb = 2;
		static const int write_nb = 2; 
		uint16_t src[write_nb];
		int prev_fwd, prev_rvs;
		
	public: 
		const char *device;
		int baud;
		char parity;
		int data_bit;
		int stop_bit;
		int device_id;
		modbus_t *ctx;
		void modbus_connection(); 
		void motor_speed(int motor_speed);
		void motor_direction(int fwd, int rvs);
};

void Modbus::modbus_connection() {
	struct timeval response_timeout;
    response_timeout.tv_sec = 10;
    response_timeout.tv_usec = 0;

	ctx = modbus_new_rtu(device, baud, parity, data_bit, stop_bit);
    if (ctx == NULL) {
        perror("Unable to create the libmodbus context\n");
    }
    
    modbus_set_response_timeout(ctx, &response_timeout);
    //modbus_set_debug(ctx, TRUE);
    
    ret = modbus_set_slave(ctx, device_id); 
    if(ret == -1){
        perror("modbus_set_slave error\n");
    }
    
    ret = modbus_connect(ctx);
    if(ret == -1){
        perror("modbus_connect error\n");
    }
}

void Modbus::motor_speed(int motor_speed) {
    // sending motor speed commands
	src[0] = motor_speed;
	src[1] = motor_speed;
	ret = modbus_write_registers(ctx, 160, write_nb, src);
	printf("motor speed sent %d %d \n", src[0], src[1]);
}

void Modbus::motor_direction(int fwd, int rvs) {
	// set them as zero first if previous fwd rvs command not the same
	if ((fwd == 0 && rvs == 0) || (prev_fwd == 0 && prev_rvs == 0) ) {
		printf("no need to convert \n");
	} else if (fwd != prev_fwd || rvs != prev_rvs) {
		src[0] = 0;
		src[1] = 0;
		ret = modbus_write_registers(ctx, 13, write_nb, src);
		printf("convert to zero first \n");
	}
	
	// sending direction commands
	src[0] = fwd;
	src[1] = rvs;
	ret = modbus_write_registers(ctx, 13, write_nb, src);
	printf("command sent %d %d \n", src[0], src[1]);
	printf("fwd: %d, pref-fwd: %d ------- rvs: %d, prev-rvs: %d \n", fwd, prev_fwd, rvs, prev_rvs);
	
	// save previous values
	prev_fwd = fwd;
	prev_rvs = rvs;
}

int main() {
    // communication config of right motor
	Modbus tsubaki_r;
	tsubaki_r.device = "/dev/ttyUSB0";  // USB path
	tsubaki_r.baud = 9600;              // communication speed
	tsubaki_r.parity = 'E';             // parity
	tsubaki_r.data_bit = 8;             // data bit length
	tsubaki_r.stop_bit = 1;             // stop bit length
	tsubaki_r.device_id = 7;            // slave address
	tsubaki_r.modbus_connection();
	sleep(0.5);
	
    // communication config of left motor
	Modbus tsubaki_l;
	tsubaki_l.device = "/dev/ttyUSB0";  // USB path
	tsubaki_l.baud = 9600;              // communication speed
	tsubaki_l.parity = 'E';             // parity
	tsubaki_l.data_bit = 8;             // data bit length
	tsubaki_l.stop_bit = 1;             // stop bit length
	tsubaki_l.device_id = 1;            // slave address
	tsubaki_l.modbus_connection();
	sleep(0.5);
	
    for(;;){	
        // motor speed
        // range between 100-3000
		int motor_speed = 1000;
		tsubaki_r.motor_speed(motor_speed);
		tsubaki_l.motor_speed(motor_speed);
		
        // motor direction commad: fwd rvs
        // forward : fwd = 1, rvs = 0
        // reverse : fwd = 0, rvs = 1
        // stop : fwd = 0, rvs = 0
		int fwd, rvs;
		fwd = 1;
        rvs = 0;
		tsubaki_r.motor_direction(fwd, rvs);
		tsubaki_l.motor_direction(fwd, rvs);
	}

    // close the connection and free the memory
    modbus_close(tsubaki_r.ctx);
    modbus_free(tsubaki_r.ctx);
    modbus_close(tsubaki_l.ctx);
    modbus_free(tsubaki_l.ctx);
}
