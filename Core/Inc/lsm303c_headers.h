/* Accelerator registers */
#define LSM303C_WHO_AM_I_ADDR             0x0F  /* device identification register */
#define LSM303C_ACT_THS_A                 0x1E
#define LSM303C_ACT_DUR_A                 0x1F
#define LSM303C_CTRL_REG1_A               0x20  /* Control register 1 acceleration */
#define LSM303C_CTRL_REG2_A               0x21  /* Control register 2 acceleration */
#define LSM303C_CTRL_REG3_A               0x22  /* Control register 3 acceleration */
#define LSM303C_CTRL_REG4_A               0x23  /* Control register 4 acceleration */
#define LSM303C_CTRL_REG5_A               0x24  /* Control register 5 acceleration */
#define LSM303C_CTRL_REG6_A               0x25  /* Control register 6 acceleration */
#define LSM303C_CTRL_REG7_A               0x26  /* Control register 6 acceleration */   
#define LSM303C_STATUS_REG_A              0x27  /* Status register acceleration */
#define LSM303C_OUT_X_L_A                 0x28  /* Output Register X acceleration */
#define LSM303C_OUT_X_H_A                 0x29  /* Output Register X acceleration */
#define LSM303C_OUT_Y_L_A                 0x2A  /* Output Register Y acceleration */
#define LSM303C_OUT_Y_H_A                 0x2B  /* Output Register Y acceleration */
#define LSM303C_OUT_Z_L_A                 0x2C  /* Output Register Z acceleration */
#define LSM303C_OUT_Z_H_A                 0x2D  /* Output Register Z acceleration */ 
#define LSM303C_FIFO_CTRL                 0x2E  /* Fifo control Register acceleration */
#define LSM303C_FIFO_SRC                  0x2F  /* Fifo src Register acceleration */

#define LSM303C_IG_CFG1_A                 0x30  /* Interrupt 1 configuration Register acceleration */
#define LSM303C_IG_SRC1_A                 0x31  /* Interrupt 1 source Register acceleration */
#define LSM303C_IG_THS_X1_A               0x32
#define LSM303C_IG_THS_Y1_A               0x33
#define LSM303C_IG_THS_Z1_A               0x34
   
#define LSM303C_IG_DUR1_A                 0x32
#define LSM303C_INT1_DURATION_A           0x33  /* Interrupt 1 DURATION register acceleration */

#define LSM303C_INT2_CFG_A                0x34  /* Interrupt 2 configuration Register acceleration */
#define LSM303C_INT2_SOURCE_A             0x35  /* Interrupt 2 source Register acceleration */
#define LSM303C_INT2_THS_A                0x36  /* Interrupt 2 Threshold register acceleration */
#define LSM303C_INT2_DURATION_A           0x37  /* Interrupt 2 DURATION register acceleration */

#define LSM303C_CLICK_CFG_A               0x38  /* Click configuration Register acceleration */
#define LSM303C_CLICK_SOURCE_A            0x39  /* Click 2 source Register acceleration */
#define LSM303C_CLICK_THS_A               0x3A  /* Click 2 Threshold register acceleration */

#define LSM303C_TIME_LIMIT_A              0x3B  /* Time Limit Register acceleration */
#define LSM303C_TIME_LATENCY_A            0x3C  /* Time Latency Register acceleration */
#define LSM303C_TIME_WINDOW_A             0x3D  /* Time window register acceleration */

/* Magnetic field Registers */
#define LSM303C_CTRL_REG1_M               0x20  /* Magnetic control register 1 */
#define LSM303C_CTRL_REG2_M               0x21  /* Magnetic control register 2 */
#define LSM303C_CTRL_REG3_M               0x22  /* Magnetic control register 3 */
#define LSM303C_CTRL_REG4_M               0x23  /* Magnetic control register 4 */
#define LSM303C_CTRL_REG5_M               0x24  /* Magnetic control register 5 */   

#define LSM303C_STATUS_REG_M              0x27  /* Magnetic status register M  */

#define LSM303C_OUT_X_L_M                 0x28  /* Output Register X magnetic field */
#define LSM303C_OUT_X_H_M                 0x29  /* Output Register X magnetic field */
#define LSM303C_OUT_Y_L_M                 0x2A  /* Output Register Y magnetic field */
#define LSM303C_OUT_Y_H_M                 0x2B  /* Output Register Y magnetic field */
#define LSM303C_OUT_Z_L_M                 0x2C  /* Output Register Z magnetic field */ 
#define LSM303C_OUT_Z_H_M                 0x2D  /* Output Register Z magnetic field */

#define LSM303C_TEMP_OUT_L_M              0x2E  /* Temperature Register magnetic field */
#define LSM303C_TEMP_OUT_H_M              0x2F  /* Temperature Register magnetic field */

#define LSM303C_INT_CFG_M                 0x30  /* Axis interrupt configuration        */
#define LSM303C_INT_SRC_M                 0x31  /* Axis interrupt source               */
#define LSM303C_INT_THS_L_M               0x32  /* Interrupt threshold L               */
#define LSM303C_INT_THS_H_M               0x33  /* Interrupt threshold M               */ 