#include ADwinGoldII.inc
rem modified by zhengyu chen, 2022-05-18
rem incorporate linear regression for model-based controller
#include .\helpfunctions.inc


#Define     MODUS_INIT            0 
#Define     MODUS_MANUEL_PISTON   1 
#Define     DELETE_CASE           2 
#Define     MODUS_MAIN_CONTROL    3 
#Define     MODUS_SHUTDOWN        4 
#Define     MODUS_EMERGENCY       10 
rem modified by zhengyu, flag to switch to model-based control
#Define     Model_based_control   12 

#Define     ZERO_OFFSET           32768
#Define     PISTON_OFFSET         32662
#Define     MAX_OUTPUT            16384
#Define     MAX_POS_OUTPUT        49152
#Define     LVDT_GLATTUNG         0.98

#Define     DIO_Zylinder          1
#Define     DIO_SpindelA          2
#Define     DIO_SpindelB          3
#Define     DIO_SpindelC          4 

#Define     WegInd_Digits          FPAR_1    'in digits
#Define     Valve_IST              FPAR_2    'in digits
#Define     KraftHTM_Digits        FPAR_3    'in digits
#Define     WegInd_Delta           FPAR_4    'in digits
#Define     WegLaser_Digits        FPAR_5    'in digits
#Define     Laser_O1               FPAR_6    'in digits
#Define     Laser_M3               FPAR_7    'not used
#Define     LVDT_2                 FPAR_8    'in mm
#Define     LVDT_3                 FPAR_9    'in mm
#Define     LVDT_1                 FPAR_10   'in mm
#Define     PorePress_Digits       FPAR_11   'in digits
#Define     AxialPress_Digits      FPAR_12   'in digits 
#Define     SidePress_Digits       FPAR_13   'in digits 
#Define     SpindelA_Digits        FPAR_14   'in digits
#Define     SpindelB_Digits        FPAR_15   'in digits
#Define     SpindelC_Digits        FPAR_16   'in digits
rem define linear model coefficients for calibration for strain-stress modle
#define     strain_linear_slope    FPAR_18
#define     strain_linear_intercept   FPAR_19

#Define     PorePressOffset        FPAR_20     'Wird von Labview beschrieben
#Define     AxialPressOffset       FPAR_21     'Wird von Labview beschrieben
#Define     SidePressOffset        FPAR_22     'Wird von Labview beschrieben
#define     vibration_amplitude_modelcontrol   fpar_23
rem define penetration velocity
#Define     penetration_vel        FPAR_24
#Define     Vorschub               FPAR_25     'Wird von Labview beschrieben
#Define     Sinusfrequenz_Vibro    FPAR_26     'Wird von Labview beschrieben 
#Define     Amplitude_Vibro        FPAR_27     'Wird von Labview beschrieben 
#Define     PreShearingOutput      FPAR_28   'in digits
#Define     start_penetrat_depth   FPAR_29 
#Define     initial_strain         FPAR_30
#DEFINE     initial_cone_position  Fpar_31 
#define     side_volume_change     FPAR_32
#define     actual_side_volume_change FPAR_33  
#Define     estimated_slope        FPAR_34
#Define     estimated_intercept    FPAR_35
#Define     Kp_Porepress           FPAR_40     'Wird von Labview beschrieben
#Define     Ki_Porepress           FPAR_41     'Wird von Labview beschrieben
#Define     Kp_Sidepress           FPAR_42     'Wird von Labview beschrieben
#Define     Ki_Sidepress           FPAR_43     'Wird von Labview beschrieben  
#Define     Kp_Axialpress          FPAR_44     'Wird von Labview beschrieben
#Define     Ki_Axialpress          FPAR_45     'Wird von Labview beschrieben
#define    Switch_freq_depth_static FPAR_46
#define    time_delay_change_freq_2  Fpar_47
#Define     PaProMin               FPAR_51     'Wird von Labview beschrieben
#Define     Huang_Su_par_1         FPAR_55     'Wird von Labview beschrieben
#Define     Huang_Su_par_2         FPAR_56     'Wird von Labview beschrieben
#Define     Huang_Su_par_3         FPAR_57     'Wird von Labview beschrieben
#Define     Huang_Su_par_4         FPAR_58     'Wird von Labview beschrieben
#Define     Huang_Su_par_5         FPAR_59     'Wird von Labview beschrieben
#Define     Huang_Su_par_6         FPAR_62     'Wird von Labview beschrieben
#Define     Huang_Su_par_7         FPAR_63     'Wird von Labview beschrieben
#define     time_delay_change_freq  Fpar_65
#define     time_delay_change_freq_1  Fpar_66
#Define     Model_PreShearing      FPAR_67
#Define     Model_Vibro            FPAR_68    
#Define     Switch_freq_depth      Fpar_69
#Define     Switch_freq_depth_1    Fpar_70
#Define     stopbutton             Fpar_71
#Define     Set_Soll_PorePressure  FPAR_72     'Wird von Labview beschrieben 
#Define     Set_Soll_AxialPressure FPAR_73     'Wird von Labview beschrieben 
#Define     Set_Soll_SidePressure  FPAR_74     'Wird von Labview beschrieben
#Define     Soll_PorePressure      FPAR_75     'Wird von Labview beschrieben 
#Define     Soll_AxialPressure     FPAR_76     'Wird von Labview beschrieben 
#Define     Soll_SidePressure      FPAR_77     'Wird von Labview beschrieben 
#Define     Taktzeit               FPAR_78
#Define     Abtastfrequenz         FPAR_79
#Define     Clock                  FPAR_80

#define     fifo_transmitflag      Par_12
#Define     Kp_Position_Vibro      PAR_1        'Wird von Labview beschrieben 
#Define     Ki_Position_Vibro      PAR_2        'Wird von Labview beschrieben 
#Define     Sinusfrequenz_preSh    PAR_4        'Wird von Labview beschrieben 
#Define     Zylinder_Move_Button   PAR_5        'Wird von Labview beschrieben
#Define     switch_frequency_button    PAR_6
#define     program_counter_interval   Par_7
#define     correlation_control_button par_9    'Wird von Labview beschrieben
rem define switch flag for vibro mode switch
rem to switch for second vibro mode
#define virboswitch_flag   par_10               
rem to switch for third vibro mode
#define virboswitch_flag_1   par_11             

rem define bug flag to indicate appearance of spike bug
rem modified by zhengyu chen 2022-05-10         
#define     BUG_flag               PAR_12   

#Define     Amplitude_PreSh        PAR_13       'Wird von Labview beschrieben 
#Define     Compass_OFF            PAR_14       'Wird von Labview beschrieben
#define     test_finish_flag       Par_15
#Define     SollMoog               PAR_17       'Wird von Labview beschrieben 
#Define     Vibro_Test_Button      PAR_21       'Wird von Labview beschrieben  
#Define     Start_Test_Button      PAR_33       'Wird von Labview beschrieben
#Define     Enable_vibro_mode_2    PAR_34
#Define     Enable_vibro_mode_3    PAR_35
#Define     Circumference_ON_Flag  PAR_37       'Wird von Labview beschrieben 
#Define     controlstart_step      Par_39   
#Define     Pore_Control_Button    PAR_40       'Wird von Labview beschrieben
#Define     Side_Control_Button    PAR_41       'Wird von Labview beschrieben 
#Define     Axial_Control_Button   PAR_42       'Wird von Labview beschrieben
#Define      total_shift           par_45 
#Define     Einfahren_Piston       PAR_46       'Wird von Labview beschrieben
#define     Switch_freq_condition_fulfil fpar_49
#Define     total_shift_max        par_53

#Define     LinearUpDown_Button    PAR_58       'Wird von Labview beschrieben
#Define     Linear_Pore_GO         PAR_59       'Wird von Labview beschrieben
#Define     Linear_Axial_GO        PAR_60       'Wird von Labview beschrieben
#Define     Linear_Side_GO         PAR_61       'Wird von Labview beschrieben
#Define     Pre_Shearing_Button    PAR_70       'Wird von Labview beschrieben
#Define     linear_calibration_flag  par_72
#Define     Circumf_Control_On     PAR_75       'Wird von Labview beschrieben

#Define     Modus                  PAR_80       'Wird von Labview beschrieben


#Define     BC5_MODEL_selection    PAR_77       'Wird von Labview beschrieben
rem mode selection for different methods to calculate additional side pressure
#Define    Huang_su_formel     0   
#Define    New_BC5_MODEL       2


rem ########################################################


rem define flag to indicate if distance array for regression is full
DIM dis_regression_full as long
rem size for regression
#define reg_Size 12
rem distance data for regressiion
DIM distance_regression_array[reg_Size] as float
rem x data for regresison
DIM time_array[reg_Size] as float
DIM deltadesired_dist as float 
rem new distance data for regressiion
DIM newdistance_regression_array[reg_Size+500] as float
rem new x data for regresison
DIM newtime_array[reg_Size+500] as float
  
rem terms for regression
DIM distance_subxmean as float
DIM distance_subymean as float
DIM distance_subxydiffpsum as float
DIM distance_subxsquaresum as float
rem pointer for regression update
DIM distpointer_vel_regression as long
  
rem intercept and slope from regression
DIM reg_para[2] as float


rem define pointer for time delay
DIM time_delay_pointer as long
DIM time_delay_pointer_1 as long
DIM time_delay_pointer_2 as long
rem pointer for initial position when we switch vibro mode
DIM switch_vibro_dist_p as long
DIM switch_vibro_dist_p_1 as long



rem ####################################################################
rem ####################################################################
rem declaration part for model controller


rem define background distance shift
DIM back_dist_shift as float
DIM back_p as long
rem define pointer for incremental calculation
DIM predict_distance_pointer as long
rem define flag to indicate array is full
DIM predict_full as long
rem define array to store estimated velocity at each step
DIM estima_vel_sub[20000] as float
'#############################################
rem define normalized velocity and controller output
DIM     normalize_vel          AS float
DIM     normalize_control      as float

rem define number of datapoints used to cover the full sine cycle with equal distance

DIM space_number as long
DIM fulldata_number as long
rem define distance and controller counter
DIM distance_counter as long
DIM controller_counter as long
rem linear regression coff
Dim linear_coff[2] as float  
Dim fittedlinear_coff[2] as float 
rem local velocity data,(velocity data at current step)
DIM vel_local as float
rem local fitted velocity 
DIM fitted_vel  as float 
rem controller cycle time
Dim controlcycletime as float
rem define controller start step
DIM controller_startstep as long

rem define local step when we start controller
DIM localstepval as long
'#define total_shift 64 
DIM distance_full_flag AS long
DIM filter_vel_full  as long
rem define select interval for velocity calculation
DIM intervalp as float
rem define size of velocity and controller output array. 20000
#define data_sizetrain 10000
rem define index for velocity and controller output array
DIM local_p as long
DIM local_c as long
rem define array to store filtered distance data
Dim filter_array[2] as float
rem define step of time offset that corresponds to maximum correlation between controller output and velocity data
DIm MAX_correlationstep as long


rem define predict distance at current step
DIM predict_dist as float
rem define desired distance at current step
DIM desired_dist as float
rem define error between desired and predict distance
DIM dist_err as float
rem define pointer for filtered distance array
DIM filter_distance_pointer as long
rem define decimate controller output and velocity
DIM decimate_WegInd_RegOut as float
DIM decimate_vel_local as float
rem define velocity deviation
DIM vel_deviation as float
rem define factor to adaptively modify desired velocity
DIM adapt_factor as float
rem define proportional strength factor for velocity deviation term, 
#define proportion_str 1300
rem define desired position
DIM soll_distance as float
REM DEFINE  distance sensor reading at first control step
DIM initialWegInd_Digits as float
rem flag to record 
DIM startcontrolflag as long

rem define distance fifo to calculate velocity
DIM realdistance_data[800] as float
rem define window size for moving average filtering
#define windowsize 1320
REM define estimated controller output
DIM estimatedWegInd_RegOut as float
REM define desired velocity at current step
DIM sollvel as float
rem define shifted desired distance
DIM shifted_desired_dist as float
rem define shifted desired distance for next step
DIM shifted_desired_dist_next as float
rem define velocity data and controller output array for two consecutive velocity data
DIM local_velocity[2] as float
DIM local_control[2] as float
DIM local_velocity_1[2] as float
DIM local_control_1[2] as float
rem pointer to update distance data for average filter
DIM pointer_average as long
rem define pointer for velocity update
DIM vel_pointer as long
DIM corr_pointer as long
rem define time offset array
DIM time_offset[2] as float

rem define full flag for estimated parameters array
DIM full_flag as long
DIM corr_pointerdist as long
rem define time offset array
DIM time_offsetdist[2] as float
DIM total_shift_maxdist as float
DIM total_shiftdist as float
rem define global pointer for drift movement of cone
DIM global_pointer as long
rem correlation part
rem define pointer for correlation calculation
DIM pointer_vel_correlation as long
DIM index as long
DIM corr_product[500] as float
rem arrays for velocity and controller output buffer in correlation calculation
DIM corr_velocity_data[500] as float
DIM corr_controlleroutput[500] as float

#Define FIFO_correlation   data_40
DIM m as long
rem define means and summations for linear regression parameter calculation
Dim subxmean as float
Dim subymean as float
DIM subxydiffpsum as float
DIM subxsquaresum as float
rem define flags to indicate data buffer is full
DIM v_fullFLAG as long
rem define desired velocity array
DIM soll_vel_array[data_sizetrain] as float
rem pointer for soll vel array
DIM soll_vel_pointer as long
rem flag to indicate if soll vel array is full
DIM soll_vel_full as float
rem switch controller flag  
DIm start_flag  as long 

DIM filter_velocity_pointer as long
rem define array for velocity deviation
DIM desired_vel_array[windowsize] as float
rem define array to store two consecutive desired velocity data after smoothing
DIM desired_vel_filter_array[2] as float
rem define pointer
DIM velpointer_average as long
rem define array for decimating desired velocity and smoothing
DIM desired_velraw[800] as float
REM define
DIM newdesired_dist as float

rem local veocity and controller data buffer, this is used to get know past 'datasize' number of data at each cycle
DIM velocity_data[data_sizetrain] as float
rem define pointer for spacing number
DIM space_pointer as long
DIM controlleroutput[data_sizetrain] as float
rem distance data buffer, which is used for moving average filtering
DIM distance_data[windowsize] as float
rem define velocity and controller output array which are used for linear regression
DIM newvelocity_data[10500] as float
DIM newcontrolleroutput[10500] as float
'DIM data_10[200000] as float
'#define data_10 velocity_data
rem define amplitude of controller output before controller starting
DIM amplitude_train as float
rem define maximum and minimum velocity 
DIM vel_max as float
DIM vel_min as float
DIM control_max as float
DIM control_min as float
rem flag to switch controller
DIM switch_flag as long
rem pointer for local velocity data buffer
DIM local_vel_pointer as long
rem pointer for veloicty data buffer for regression
DIM pointer_vel_regression as long
rem define array to record damping amplitude of sine movement before switching to model-based controller
DIM damping_amplitude as float
rem define dampling pointer
DIM damping_pointer as long
rem define pointer for desired distance update
DIM desired_distpointer as long
rem define flag to indicate if desired velocity array is full
DIM desired_vel_fullflag as long 
rem array for maximum and minimum distance calculation for one period
DIM distance_max[2] as float
DIM distance_min[2] as float
rem define control cycle
DIM control_cycle as long
rem define index for recalculate desired distance 
DIM local_index as long 
rem define pointer for acceleration array
DIM local_acceleration as float
rem define flag to indicate acceleration array is full
DIM accleration_full as long
rem size of accelration data array
#define acceleration_size 150 
rem define acceleration array
DIM accleration[data_sizetrain] as float
rem implemented by zhengyu chen 2022-05-18#######
DIM Programm_counter                As Long    
Dim xmean,ymean as float
rem define amplitude calibration flag
DIM amplitude_calibration_flag as long
rem define estimated parameters array and pointer
DIM estimated_slope_array[data_sizetrain] as float
DIM estimated_intercept_array[data_sizetrain] as float
DIM estimated_parameter_pointer as long
rem define pointer for distance sensor reading
DIM NEWpointer_average as long
rem define indice for peak distance measurement
DIM maxindice as float
DIM damping_amplitudeold as float
rem define button to enable model-based control
DIM enable_model_control as long  
rem define pointer for distance data
DIM dist_pointer as long
rem define first vibration frequency
DIM original_vib_freq as float
DIM predict_distance_array[data_sizetrain] as float
rem define predict distance pointer
DIM pre_dist_p as long
rem define controller output array to deal with spike problem
DIM weg_ingoutarray[2] as float
rem define local pointer
DIM enter_cycle_number as long

dim new_desired_dist as float
dim newinitialWegInd_Digits as float
rem ####################################################################
rem ####################################################################


rem declaration for side volume 
DIM initial_side_volume as float    
DIM side_volume_pointer as long




rem 


rem modified by zhengyu chen, increase Fifo buffer size 
rem date: 2022.04.29
rem   data_1 : 20000, original size:5000
rem   data_12: 20000, original size: 5000
rem   data_50: 120000, original size: 80000
rem   data_2; 20000, original size: 5000


DIM data_1[30000]   as Long as FIFO  'zum R232 auslesen
DIM data_12[30000]  as Long          'zum R232 auslesen
rem define fifo array to transmit correlation results
DIM data_40[120000] as Float as Fifo
DIM data_50[160000] as Float as Fifo  'main FIFO output 

DIM data_2[30000] as Float as Fifo
DIM data_3[10000] as Float as Fifo
DIM data_4[10000] as Float as Fifo
DIM data_5[10000] as Float as Fifo
#Define FIFO_RS_RAW     data_1  
#Define FIFO_RS_SYNCED  data_12  
#Define FIFO_MAIN_OUT   data_50


rem define local time step
DIM localtimestep as long

rem define array to store additional side pressure at current and last time step
DIM Additionalside_pressure_array[2] as float   
rem define intial volume 
DIM initial_volume as float
rem height of cylinder
DIM height_cylinder as float
#Define pi 3.1415926
rem initial strain
DIM r_0 as float
rem initial depth
DIM weginitial_depth as float
rem strain ration
DIM delta_r as float
rem define largest penetration depth for calibration with BC5
DIM lowestWeg as float
DIM lowestWeg_phy as float
rem define actual volume
DIM Actual_volume as float
rem define local counter
DIM local_counter as long
dim final_switch as long

rem define initial position of piston
DIM initial_piston_position as float 
DIM p as long

rem define array to store program counter value at two consecutive steps
DIM programm_cycle[2] as float








rem clock cycle that FIfo write the data
DIM fifotime as long
rem clock cycle that starts to read the data
DIM readtdatatime as long
#define null 0

'Positionsregler        
DIM WegInd_DeltaSum                 As Float             'Integrationsanteil        
DIM WegInd_RegOut                   As Float             'Ausgabewert Regler
DIM Soll_Position_Regelungswert     As Float             'Ãœbergabewert zwischen den Cases

DIM Kp_Position                     As Float             'P-Regler 
DIM Ki_Position                     As Float             'I-Regler

'Porendruck Regler
DIM PorePress_Delta                 As Float             'Delta aus Soll-Wert und Ist-Wert
DIM PorePress_DeltaSum              As Float             'Integrationsanteil         
DIM PorePress_RegOut                As Float             'Ausgabewert Regler

'Seitendruck Regler
DIM SidePress_Delta                 As Float             'Delta aus Sollwert und Istwert
DIM SidePress_DeltaSum              As Float             'Integrationsanteil      
DIM SidePress_RegOut                As Float             'Ausgabewert Regler

'Axialdruck Regler
DIM AxialPress_Delta                As Float             'Delta aus Soll-Wert und Ist-Wert
DIM AxialPress_DeltaSum             As Float             'Integrationsanteil        
DIM AxialPress_RegOut               As Float             'Ausgabewert Regler

'Axial/Side/Pore GlÃ¤ttung
DIM glattung_index                  As Long

DIM Array_PoreP[20]                 As Float
DIM Array_Axial[20]                 As Float
DIM Array_SideP[20]                 As Float

DIM Array_AxialPress[20]            As Float
DIM Array_SidePress[20]             As Float
DIM Array_PorePress[20]             As Float

DIM Array_SpindelA[20]              As Float
DIM Array_SpindelB[20]              As Float
DIM Array_SpindelC[20]              As Float

'Umfangsregelung
DIM LVDT_MAX                        As Float
DIM Side_Addtion_Phys               As Float
DIM Side_Addtion_Digits             As Float
DIM Umfang_Offset_Flag              As Long
DIM Umfang1_Offset                  As Float
DIM Umfang2_Offset                  As Float
DIM Umfang3_Offset                  As Float
DIM Umfang1_porevolume              As Float
DIM Umfang1_sidevolume              As Float
DIM Umfang1_axialvolume              As Float
rem define actual side volume change
DIM actual_sidevolume               As float
DIM LVDT_1_Old                      As Float
DIM LVDT_2_Old                      As Float
DIM LVDT_3_Old                      As Float
DIM Abschaltung_Reglungs_Wert       As Float

'Model
DIM Vibro_internal_counter          As Float
DIM PreSh_internal_counter          As Float



DIM Spitzendruck_D                  As Float
DIM Porendruck_D                    As Float

'RS232 auslesen 
DIM cnt as Long
DIM ret_val, val as Long
DIM z, j as long
DIM dummy as long
DIM sync1, sync2, checksum as long
DIM lastSeqNo as Long
DIM dataValues[5000] as Float
DIM actCnt_ as Long
DIM fifoused_ as Float
DIM synced_, resync_ as Float
DIM dataValid_ as Long
DIM avisaroType as Long

'DIM StartingPositionofPiston as Float 'Debug
'DIM heightofthepiston_Digits as Float'Debug
'DIM V_physical as Float 'Debug   V     'Debug
'DIM dV_Physical as Float 'Debug dV
'DIM dV_from_pump  as Float  'Debug


 
Function average_array(array[]) as float
  average_array = (array[1]+array[2]+array[3]+array[4]+array[5]+array[6]+array[7]+array[8]+array[9]+array[10]+array[11]+array[12]+array[13]+array[14]+array[15]+array[16]+array[17]+array[18]+array[19]+array[20]) / 20
EndFunction

Init:
  Digout(1,1)                                                         ' Freigabe der Lasersensoren
  Conf_DIO(0011b)                                                     ' DigOut 0-15 / DigIn 15-31  
  Processdelay    = 60000                                             ' entspricht  (1 Sekunde / Taktzyklus 3,333 ns * Processdelay)
  Abtastfrequenz  = 1 / (3.333333333*10^-9 * Processdelay)            ' Bei einem Processdelay von 60000 ist die Abtastfrequenz = 5000 Hz
  Taktzeit        = 1 /  Abtastfrequenz                               ' Bei einem Processdelay von 60000 Abtastzeit alle 2 * 10 ^(-4) Sekunden
  Modus = 0
  Clock = 0
  Start_Test_Button = 0
  
  test_finish_flag = 0
  
  
  
  '------------------------------------------------------------------------------------------------------------------------------------------ 
  '------------------------------------------------------------------------------------------------------------------------------------
  '  Model-based controller initialization part 
  '------------------------------------------------------------------------------------------------------------------------------------
  '------------------------------------------------------------------------------------------------------------------------------------
 
  
  rem initialize enable second and third vibro mode button
  Enable_vibro_mode_2 = 0
  Enable_vibro_mode_3 = 0
  rem initialize cotroller status button
  controlstart_step = 0
  rem initialize switch controller flag 
  switch_flag = 0
  rem initialize index for distance data array
  pointer_average = 0
  rem initialize index for smoothed data array with regression fitting
  NEWpointer_average = 0
  rem initialize time offset
  
  p = 1
  rem initialize index counted from when we switch to model controller
  local_index = 1
  rem initialize amplitude of sine controller output at traning stage
  amplitude_train = ((fpar_23-Sinusfrequenz_Vibro-10)*0.2- 1.15843465)/(2.67370297e-4) + 2500  

  rem initialize switch frequency button
  switch_frequency_button  = 0
  
  REM intialize penetration velocity
  penetration_vel = fpar_24
  
  
  rem initialize time delay
  total_shift = 0
  rem initialize linear regression parameters
  linear_coff[1] =    -0.000301003 
  linear_coff[2] =     12.9918  
  rem initialize bug flag rem calculate number of points for full sine cycle
  fulldata_number  = 1*Abtastfrequenz/Sinusfrequenz_Vibro
  rem initialie flag to indicate spike bug
  BUG_flag = 0
  rem define pointer for background shift update
  back_p = 1
  rem initialize cycle number when we switch on modelcontroller
  enter_cycle_number = 0
  rem initialize estimated parameter array pointer
  estimated_parameter_pointer = 1
  rem initialie flag to indicate if array is full
  full_flag = 0
  rem local pointer for controller output data array
  local_c = 1
  rem flag to indicate if distance data array is full
  distance_full_flag = 0
  rem initialize time delay pointer 
  time_delay_pointer = 0
  
  time_delay_pointer_1 = 0
  time_delay_pointer_2 = 0
  rem initialize vibro switch flag 
  virboswitch_flag = 0
  virboswitch_flag_1 = 0
  rem initialize desired velocity array pointer
  
 
  rem initialize amplitude calibration flag 
  amplitude_calibration_flag = 0
  rem initialize space pointer
  space_pointer = 0
  rem initialize space number
  space_number = 1
  rem initialize controller stop button
  stopbutton = 0
  rem initialize pointer for predict distance array
  pre_dist_p = 1
  pointer_vel_correlation = 0
  rem initialize terms and pointers for fitted parameter calculation
  local_vel_pointer = 1
  filter_distance_pointer = 1
  soll_vel_pointer = 1
  soll_vel_full = 0
  pointer_vel_regression = 1
  v_fullFLAG = 0
  subxmean = 0
  subymean = 0
  subxydiffpsum = 0
  subxsquaresum = 0
  rem initialize pointer to record starting position when we change to another vibro mode
  switch_vibro_dist_p = 0
  switch_vibro_dist_p_1 = 0
  rem intialize velocity pointer
  vel_pointer = 1
  rem intialize correlation pointer
  corr_pointer = 1
  damping_pointer = 1
  rem number of cycles when we start model controller mode
  control_cycle = 0
  rem pointer for desired distance calculation
  desired_distpointer = 1
  rem initialie original freq
  original_vib_freq  = 0
  final_switch = 0
  corr_pointerdist = 1
  dist_pointer = 0
  rem global pointer when we switch to model controller
  global_pointer = 1
  rem flag to indicate acceleration array is full
  accleration_full = 0
  rem initialize correlation array
  for m = 1 to fulldata_number
    corr_product[m] = 0
        
  next m
  
  rem initialize for the regression calculation for smoothing distance

  
  dis_regression_full = 0
  
  distpointer_vel_regression = 1
  
  distance_subxmean = 0
  distance_subymean = 0
  distance_subxydiffpsum = 0
  distance_subxsquaresum = 0
  
  side_volume_pointer = 1
  
  local_counter = 0
  localtimestep = 1
  rem initialize p
  p = 1
  '------------------------------------------------------------------------------------------------------------------------------------------ 
  '------------------------------------------------------------------------------------------------------------------------------------------ 
 
  
  Kp_Porepress  = 40       
  Ki_Porepress  = 0.2
  Kp_Sidepress  = 40          
  Ki_Sidepress  = 0.2         
  Kp_Axialpress = 40       
  Ki_Axialpress = 0.2

  Huang_Su_par_1 = -26218
  Huang_Su_par_2 = 75512
  Huang_Su_par_3 = -85173     
  Huang_Su_par_4 = 47543      
  Huang_Su_par_5 = -13605    
  Huang_Su_par_6 = 2045     
  Huang_Su_par_7 = 0  
  
  Soll_PorePressure  = ZERO_OFFSET  
  Soll_AxialPressure = ZERO_OFFSET   
  Soll_SidePressure  = ZERO_OFFSET
  Set_Soll_SidePressure  = ZERO_OFFSET
  Set_Soll_AxialPressure = ZERO_OFFSET
  Set_Soll_PorePressure = ZERO_OFFSET
  
  Kp_Position_Vibro      = 150
  Ki_Position_Vibro      = 0
  Sinusfrequenz_Vibro    = 10
  Sinusfrequenz_preSh    = 0
  Zylinder_Move_Button   = 0
  Amplitude_Vibro        = 3
  Amplitude_PreSh        = 0       
  SollMoog               = ZERO_OFFSET
  
  Programm_counter       = 1  
  Circumference_ON_Flag  = 0 
  Pore_Control_Button    = 0
  Side_Control_Button    = 0
  Axial_Control_Button   = 0
  Einfahren_Piston       = 0
  LinearUpDown_Button    = 0
  Linear_Pore_GO         = 0
  Linear_Axial_GO        = 0
  Linear_Side_GO         = 0
  Pre_Shearing_Button    = 0
  Modus                  = 0
                                   
  'Positionsregler fÃ¼rs Position halten
  Kp_Position = 5

  WegInd_DeltaSum     = 0
  AxialPress_DeltaSum = 0 
  SidePress_DeltaSum  = 0 
  PorePress_DeltaSum  = 0
        
  WegInd_RegOut     = PISTON_OFFSET
  AxialPress_RegOut = ZERO_OFFSET
  SidePress_RegOut  = ZERO_OFFSET     
  PorePress_RegOut  = ZERO_OFFSET
        
  glattung_index = 1
  
  PreSh_internal_counter = 0
  Vibro_internal_counter = 0
  
  'FÃ¼r die Umfangsregelung
  Umfang_Offset_Flag  = 0
  LVDT_Max            = 0
  Side_Addtion_Digits = 0
  Side_Addtion_Phys   = 0
  Umfang1_Offset      = 0
  Umfang2_Offset      = 0
  Umfang3_Offset      = 0
  Umfang1_porevolume  = 0
  Umfang1_sidevolume  = 0
  Umfang1_axialvolume = 0
  Abschaltung_Reglungs_Wert = 0
  
  
  'StartingPositionofPiston = 0 'Debug
  'heightofthepiston_Digits = 0'Debug
  'V_physical = 0  'Debug   V     'Debug
  'dV_Physical = 0  'Debug dV
  'dV_from_pump = 0 ' Debug
  
  
 
  'RS385 Schnittstelle
  RS_RESET()
  RS_INIT(1,408060,0,8,0,3)
  RS485_SEND(1,0)
  
  FIfO_clear(1)  
  
  cnt = 0
  actCnt_ = 0      
  LastSeqNo = 1
  synced_ = 0
  fIfoused_ = 0
  resync_ = 0
  dataValid_ = 0  
  
  
  
  
  rem \\\\\\\\\\\\\\\\\ new calibration model
Event:
  Clock = Clock + Taktzeit / 100
  rem modified by Zhengyu Chen 2022.04.26
  rem record clock cycle that start to read data
  
  readtdatatime = Read_Timer()
  rem modified by Zhengyu Chen 2022.04.26
  
  'Auslesen Kraft externer Z - Sensor (Analog 3)
  Set_Mux1(00001b)                                'Setzt den Multiplexer I auf den Analogeingang 3 VerstÃ¤rkungsfaktor 1
  IO_Sleep(200)                                   'Rem IO-ZugrIff fÃ¼r  200 us (MUX-Einschwingzweit) unterbrechen
  Start_Conv(01b)                                 'Start der AD Wandlung (01b) = Converter 1
  Wait_EOC(01b)                                   'Wartet bis AD Wandlung am Converter 1 abgeschlossen 
  KraftHTM_Digits = Read_ADC(01b) 
  
  'Auslesen induktiver Wegsensor (Analog 4) 
  Set_Mux2(00001b)                                'Setzt den Multiplexer II auf den Analogeingang 4 VerstÃ¤rkungsfaktor 1
  IO_Sleep(200)                                   'Rem IO-ZugrIff fÃ¼r  200 us (MUX-Einschwingzweit) unterbrechen
  Start_Conv(10b)                                 'Start der AD Wandlung (10b) = Converter 2
  Wait_EOC(10b)                                   'Wartet bis AD Wandlung am Converter 2 abgeschlossen
  WegInd_Digits  = Read_ADC(10b)                  'Digits 
  
  'NEW LVDT 2 (Analog 5)
  Set_Mux1(00010b)                                'Setzt den Multiplexer I auf den Analogeingang 5 VerstÃ¤rkungsfaktor 1
  IO_Sleep(200)                                   'Rem IO-ZugrIff fÃ¼r  200 us (MUX-Einschwingzweit) unterbrechen
  Start_Conv(01b)                                 'Start der AD Wandlung (01b) = Converter 1
  Wait_EOC(01b)                                   'Wartet bis AD Wandlung am Converter 1 abgeschlossen
  LVDT_2  = ((10 / 32768) * Read_ADC(01b))-10                   'Digits         
  
  'Laser Oben 1 (Analog 6) 
  Set_Mux2(00010b)                                'Setzt den Multiplexer II auf den Analogeingang 6 VerstÃ¤rkungsfaktor 1
  IO_Sleep(200)                                   'Rem IO-ZugrIff fÃ¼r  200 us (MUX-Einschwingzweit) unterbrechen
  Start_Conv(10b)                                 'Start der AD Wandlung (10b) = Converter 2
  Wait_EOC(10b)                                   'Wartet bis AD Wandlung am Converter 2 abgeschlossen                  
  Valve_IST = Read_ADC(10b)                       'Digits
  
  'NEW LVDT 1 (Analog 7)
  Set_Mux1(00011b)                                'Setzt den Multiplexer I auf den Analogeingang 7 VerstÃ¤rkungsfaktor 1
  IO_Sleep(200)                                   'Rem IO-ZugrIff fÃ¼r  200 us (MUX-Einschwingzweit) unterbrechen
  Start_Conv(01b)                                 'Start der AD Wandlung (01b) = Converter 1
  Wait_EOC(01b)                                   'Wartet bis AD Wandlung am Converter 1 abgeschlossen
  LVDT_1  = ((10 / 32768) * Read_ADC(01b))-10     'in [mm]                    
  
  'OLD LVDT 2 (Analog 8) 
  Set_Mux2(00011b)                                'Setzt den Multiplexer II auf den Analogeingang 8 VerstÃ¤rkungsfaktor 1
  IO_Sleep(200)                                   'Rem IO-ZugrIff fÃ¼r  200 us (MUX-Einschwingzweit) unterbrechen
  Start_Conv(10b)                                 'Start der AD Wandlung (10b) = Converter 2
  Wait_EOC(10b)                                   'Wartet bis AD Wandlung am Converter 2 abgeschlossen
  'LVDT_2  = ((10 / 32768) * Read_ADC(10b))-10     'in [mm]                    
  
  'LVDT 3 (Analog 9)
  Set_Mux1(00100b)                                'Setzt den Multiplexer I auf den Analogeingang 9 VerstÃ¤rkungsfaktor 1
  IO_Sleep(200)                                   'Rem IO-ZugrIff fÃ¼r  200 us (MUX-Einschwingzweit) unterbrechen
  Start_Conv(01b)                                 'Start der AD Wandlung (01b) = Converter 1
  Wait_EOC(01b)                                   'Wartet bis AD Wandlung am Converter 1 abgeschlossen
  LVDT_3   = ((10 / 32768) * Read_ADC(01b))-10    'in [mm]    
  
  'OLD LVDT 1 (Analog 10)
  Set_Mux2(00100b)                                'Setzt den Multiplexer II auf den Analogeingang 10 VerstÃ¤rkungsfaktor 1
  IO_Sleep(200)                                   'Rem IO-ZugrIff fÃ¼r  200 us (MUX-Einschwingzweit) unterbrechen
  Start_Conv(10b)                                 'Start der AD Wandlung (10b) = Converter 2
  Wait_EOC(10b)                                   'Wartet bis AD Wandlung am Converter 2 abgeschlossen
  'LVDT_1  = ((10 / 32768) * Read_ADC(10b))-10     'in [mm]                     
  
  'Porendruck (Analog 11)
  Set_Mux1(00101b)                                'Setzt den Multiplexer I auf den Analogeingang 11 VerstÃ¤rkungsfaktor 1
  IO_Sleep(200)                                   'Rem IO-ZugrIff fÃ¼r  200 us (MUX-Einschwingzweit) unterbrechen
  Start_Conv(01b)                                 'Start der AD Wandlung (01b) = Converter 1
  Wait_EOC(01b)                                   'Wartet bis AD Wandlung am Converter 1 abgeschlossen
  PorePress_Digits  =  Read_ADC(01b)             
    
  'Axialdruck (Analog 12)
  Set_Mux2(00101b)                                'Setzt den Multiplexer II auf den Analogeingang 12 VerstÃ¤rkungsfaktor 1
  IO_Sleep(200)                                   'Rem IO-ZugrIff fÃ¼r  200 us (MUX-Einschwingzweit) unterbrechen
  Start_Conv(10b)                                 'Start der AD Wandlung (10b) = Converter 2
  Wait_EOC(10b)                                   'Wartet bis AD Wandlung am Converter 2 abgeschlossen
  AxialPress_Digits = Read_ADC(10b)                    
  
  'SidePress (Analog 13)
  Set_Mux1(00110b)                                'Setzt den Multiplexer I auf den Analogeingang 13 VerstÃ¤rkungsfaktor 1
  IO_Sleep(200)                                   'Rem IO-ZugrIff fÃ¼r  200 us (MUX-Einschwingzweit) unterbrechen
  Start_Conv(01b)                                 'Start der AD Wandlung (01b) = Converter 1
  Wait_EOC(01b)                                   'Wartet bis AD Wandlung am Converter 1 abgeschlossen
  SidePress_Digits  = Read_ADC(01b)               
  
  'SpindelA (Analog 14)
  Set_Mux2(00110b)                                'Setzt den Multiplexer II auf den Analogeingang 14 VerstÃ¤rkungsfaktor 1
  IO_Sleep(200)                                   'Rem IO-ZugrIff fÃ¼r  200 us (MUX-Einschwingzweit) unterbrechen
  Start_Conv(10b)                                 'Start der AD Wandlung (10b) = Converter 2
  Wait_EOC(10b)                                   'Wartet bis AD Wandlung am Converter 2 abgeschlossen         
  SpindelA_Digits   = Read_ADC(10b)

  'SpindelB (Analog 15)
  Set_Mux1(00111b)                                'Setzt den Multiplexer I auf den Analogeingang 15 VerstÃ¤rkungsfaktor 1
  IO_Sleep(200)                                   'Rem IO-ZugrIff fÃ¼r  200 us (MUX-Einschwingzweit) unterbrechen
  Start_Conv(01b)                                 'Start der AD Wandlung (01b) = Converter 1
  Wait_EOC(01b)                                   'Wartet bis AD Wandlung am Converter 1 abgeschlossen   
  SpindelB_Digits    = Read_ADC(01b)
  
  'SpindelC (Analog 16)
  Set_Mux2(00111b)                                'Setzt den Multiplexer II auf den Analogeingang 16 VerstÃ¤rkungsfaktor 1
  IO_Sleep(200)                                   'Rem IO-ZugrIff fÃ¼r  200 us (MUX-Einschwingzweit) unterbrechen
  Start_Conv(10b)                                 'Start der AD Wandlung (10b) = Converter 2
  Wait_EOC(10b)                                   'Wartet bis AD Wandlung am Converter 2 abgeschlossen  
  SpindelC_Digits     = Read_ADC(10b)
  
  rem if we finish the test use manuel control mode
  if (test_finish_flag = 1) then 
    Modus = 1
  endif
  
  rem parameter setup for model controller
  parameter_setup(Switch_freq_depth_static,Enable_vibro_mode_2,Enable_vibro_mode_3,Switch_freq_depth_1,time_delay_pointer_1,time_delay_change_freq_1,time_delay_change_freq,damping_amplitude,penetration_vel,virboswitch_flag_1,virboswitch_flag,time_delay_pointer,Switch_freq_depth,intervalp,switch_frequency_button,linear_calibration_flag,WegInd_Digits ,controlstart_step, predict_dist,programm_cycle,stopbutton,total_shift ,WegInd_RegOut,fulldata_number,Abtastfrequenz,original_vib_freq,PorePressOffset,AxialPressOffset,SidePressOffset,Sinusfrequenz_Vibro,Programm_counter) 
 
  
 
  
  If (glattung_index < 20) Then  'GlÃ¤ttung Signale
    
    Array_PoreP[glattung_index] = PorePress_Digits - PorePressOffset
    Array_SideP[glattung_index] = SidePress_Digits - SidePressOffset 
    Array_Axial[glattung_index] = AxialPress_Digits - AxialPressOffset 
    
    Array_SpindelA[glattung_index] = SpindelA_Digits
    Array_SpindelB[glattung_index] = SpindelB_Digits
    Array_SpindelC[glattung_index] = SpindelC_Digits
    
    Array_AxialPress[glattung_index] = AxialPress_Delta
    Array_SidePress[glattung_index] = SidePress_Delta
    Array_PorePress[glattung_index] = PorePress_Delta
    
    Inc glattung_index 'Increment
  Else
    
    Array_PoreP[20] = PorePress_Digits - PorePressOffset
    Array_SideP[20] = SidePress_Digits - SidePressOffset 
    Array_Axial[20] = AxialPress_Digits - AxialPressOffset 
    
    Array_SpindelA[20] = SpindelA_Digits
    Array_SpindelB[20] = SpindelB_Digits
    Array_SpindelC[20] = SpindelC_Digits
    
    Array_AxialPress[20] = AxialPress_Delta
    Array_SidePress[20] = SidePress_Delta
    Array_PorePress[20] = PorePress_Delta
    
    
    glattung_index = 1
  EndIf
  
  
  PorePress_Digits   = average_array(Array_PoreP)
  SidePress_Digits   = average_array(Array_SideP)
  AxialPress_Digits  = average_array(Array_Axial)
  
  SpindelA_Digits   = average_array(Array_SpindelA)
  SpindelB_Digits   = average_array(Array_SpindelB)
  SpindelC_Digits   = average_array(Array_SpindelC)
  
  AxialPress_Delta  = average_array(Array_AxialPress)
  SidePress_Delta   = average_array(Array_SidePress)
  PorePress_Delta   = average_array(Array_PorePress)
   
  If ((Umfang_Offset_Flag = 0) and (Circumf_Control_On = 1)) then 
          
    Umfang_Offset_Flag = 1
    Umfang1_Offset = LVDT_1
    Umfang2_Offset = LVDT_2
    Umfang3_Offset = LVDT_3
    rem record initial side volume
  
    Umfang1_sidevolume = SpindelB_Digits
    
  EndIf
  rem calculate volumetric deformation
  actual_sidevolume = SpindelB_Digits - Umfang1_sidevolume
  
  rem record volume deformation
  LVDT_1 = LVDT_1 - Umfang1_Offset
  LVDT_2 = LVDT_2 - Umfang2_Offset
  LVDT_3 = LVDT_3 - Umfang3_Offset
  
  'GlÃ¤ttung der LVDT Signale 
  LVDT_1 = (LVDT_1*(1-LVDT_GLATTUNG)) + (LVDT_1_Old * LVDT_GLATTUNG)
  LVDT_1_Old = LVDT_1
  
  LVDT_2 = (LVDT_2*(1-LVDT_GLATTUNG)) + (LVDT_2_Old * LVDT_GLATTUNG)
  LVDT_2_Old = LVDT_2
  
  LVDT_3 = (LVDT_3*(1-LVDT_GLATTUNG)) + (LVDT_3_Old * LVDT_GLATTUNG)
  LVDT_3_Old = LVDT_3

  
  
  
  
  
  
  Selectcase Modus
    Case MODUS_INIT
      
      Digout(0,1)                      'Freigabe fÃ¼r Moogventil
      Digout(1,1)                      'Freigabe fÃ¼r Lasersensoren   
      DAC(DIO_Zylinder, PISTON_OFFSET) 'Zylinder not moving (only slightly up)
      DAC(DIO_SpindelA, ZERO_OFFSET)   'SpindelA not moving
      DAC(DIO_SpindelB, ZERO_OFFSET)   'SpindelB not moving
      DAC(DIO_SpindelC, ZERO_OFFSET)   'SpindelC not moving
      
      Soll_Position_Regelungswert = WegInd_Digits   'Ãœbergabe an den nÃ¤chsten Case
     
    Case MODUS_MANUEL_PISTON  '--------------------------------------------------------------------------------------------------------------------------
      
      Digout(0,1)       'Freigabe fÃ¼r Moogventil
      Digout(1,1)       'Freigabe fÃ¼r Lasersensoren (Dickenmessung der Probe)     
      
      DAC(DIO_SpindelA, ZERO_OFFSET)   'SpindelA not moving
      DAC(DIO_SpindelB, ZERO_OFFSET)   'SpindelB not moving
      DAC(DIO_SpindelC, ZERO_OFFSET)   'SpindelC not moving 
      
      If (SollMoog > 40000) Then SollMoog = 40000 'Capping max posive velocity
      If (SollMoog < 26000) Then SollMoog = 26000 'Capping max negative velocity
      
      If (Zylinder_Move_Button = 1) Then 'Manuelle Bedienung des Kolbens
       
        DAC(DIO_Zylinder,SollMoog)
        
        Soll_Position_Regelungswert = WegInd_Digits 'speichern der letzten SOLL Position
      
      Else ' Halten der Position
      
        WegInd_Delta = WegInd_Digits - Soll_Position_Regelungswert 'auf letzte Position geregelt
        WegInd_RegOut = (Kp_Position * WegInd_Delta)
        
        If (WegInd_RegOut < -MAX_OUTPUT)  Then WegInd_RegOut = -MAX_OUTPUT
        If (WegInd_RegOut > MAX_OUTPUT)  Then WegInd_RegOut = MAX_OUTPUT
  
        WegInd_RegOut= WegInd_RegOut + ZERO_OFFSET
        DAC(DIO_Zylinder,WegInd_RegOut)
      
      EndIf
      
      
      
      
      
      
      
      
      rem ////////////////////////////////////////////////////////////////////////////  
      rem model/based controller with simple linear regression model, implemented by zhengyu chen,
      rem latest modified date> 2022.06.08    \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
    Case Model_based_control
      
      Digout(0,1)       'Freigabe fÃƒÂ¼r Moogventil
      Digout(1,1)       'Freigabe fÃƒÂ¼r Lasersensoren
      
      rem record time number for sine wave
      If (Abtastfrequenz/Sinusfrequenz_Vibro > Vibro_internal_counter) then 
        INC Vibro_internal_counter
      Else
        Vibro_internal_counter = 0
      EndIf
      
  
     
        
     
 
      rem ////////////////////////////////////////////////////////////////////////////  
      rem model/based controller with simple linear regression model , implemented by zhengyu chen,
      rem latest modified date> 2022.06.07    \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ 
    
      
      rem generate controller output 
      if ((controlstart_step = 0)   )  then
        rem switch between vibro starting mode and static starting mode, depends on frequecy
        if (Sinusfrequenz_Vibro  < 25) then
          
          
          rem keep position of piston
          
          WegInd_Delta = WegInd_Digits - Soll_Position_Regelungswert 'auf letzte Position geregelt
          WegInd_RegOut = (Kp_Position * WegInd_Delta)
        
          If (WegInd_RegOut < -MAX_OUTPUT)  Then WegInd_RegOut = -MAX_OUTPUT
          If (WegInd_RegOut > MAX_OUTPUT)  Then WegInd_RegOut = MAX_OUTPUT
  
          WegInd_RegOut= WegInd_RegOut + ZERO_OFFSET
        
          
        else
                    
          amplitude_train = 2600    
          WegInd_RegOut = amplitude_train*sin(2*3.1415926*(Sinusfrequenz_Vibro/Abtastfrequenz)* Vibro_internal_counter) + 32768
    
        endif

      endif  
      rem ########################################
                         
      rem calculate desired velocity with smoothing and fitted distance data
                
      distance_data_regression (time_array,vel_local,intervalp,data_sizetrain,realdistance_data,dist_pointer,dis_regression_full,pointer_average, reg_Size,distance_regression_array,reg_para,newdistance_regression_array,newtime_array,distance_subxmean,distance_subymean,distance_subxydiffpsum,distance_subxsquaresum,distpointer_vel_regression,Abtastfrequenz,Sinusfrequenz_Vibro,WegInd_Digits,pointer_average) 
      ' 
      rem record smoothed distance data 
      Distance_data_smooth(distance_data,realdistance_data,intervalp,filter_array,Programm_counter,NEWpointer_average,windowsize,reg_para[2]) 
      
     
     
                        
 
      rem calculate maximum and minimum for normalization
      max_min_calcul(fulldata_number,vel_pointer,local_velocity,local_velocity_1,local_control,local_control_1,vel_max,vel_min,control_max,control_min,vel_local,WegInd_RegOut)   
                    
          
     
                      
              

      rem calculate time offset
     
     
      timeoffset(control_cycle,original_vib_freq, Sinusfrequenz_Vibro,correlation_control_button,fulldata_number,corr_pointer,total_shift,time_offset,total_shift_max)
          
   
    
      
     
     
      
      
      ' when we start the controller and (Start_Test_Button = 1)
      if ((controlstart_step = 1)    )  then
        
      
        
        
        rem calculate acceleration of cone movement
                         
        acceleration_cal(accleration_full,intervalp,local_acceleration,acceleration_size,data_sizetrain,accleration,velocity_data,local_vel_pointer) 
                         
                  
      
        if (switch_flag = 1) then
          estimated_parameters_justification(Start_Test_Button,full_flag,estimated_parameter_pointer,linear_coff,estimated_slope_array,estimated_intercept_array)  
        endif
      
      
        rem calculate desired distance
      
        
        desired_distance(enter_cycle_number,switch_vibro_dist_p_1,virboswitch_flag_1,virboswitch_flag,switch_vibro_dist_p,desired_dist,reg_para[2],switch_flag,total_shiftdist, original_vib_freq,local_index,penetration_vel,global_pointer,desired_distpointer,total_shiftdist,initialWegInd_Digits,fulldata_number,Abtastfrequenz,1*damping_amplitude,Sinusfrequenz_Vibro,Vibro_internal_counter)
 
        rem main model control part
       
        model_control(switch_vibro_dist_p,switch_vibro_dist_p_1,controlleroutput,local_c,final_switch,penetration_vel,virboswitch_flag, virboswitch_flag_1,damping_amplitude,vel_local,data_sizetrain,predict_distance_array,pre_dist_p,weg_ingoutarray,enter_cycle_number,desired_dist,intervalp,linear_coff,Abtastfrequenz,Sinusfrequenz_Vibro,Vibro_internal_counter,total_shiftdist, WegInd_RegOut,sollvel,initialWegInd_Digits,predict_dist,estimatedWegInd_RegOut,total_shift,switch_flag,global_pointer, reg_para[2],vel_deviation, dist_err, proportion_str) 
      
        
       
                                    
      endif
       
      rem number of datapoints for one period
      fulldata_number  = 1*Abtastfrequenz/Sinusfrequenz_Vibro
      rem if we switch to the controller, firstly use safe parameters, after dataset for regression is complete, then we 
      rem start to do linear regression
      
      sub_blockcalculation(estimated_slope_array,estimated_intercept_array,estimated_parameter_pointer,Abtastfrequenz,Sinusfrequenz_Vibro,v_fullFLAG,local_vel_pointer, data_sizetrain,velocity_data,linear_coff,newvelocity_data,newcontrolleroutput,subxmean,subymean,subxydiffpsum,subxsquaresum,pointer_vel_regression ,WegInd_RegOut, vel_local, total_shift,fulldata_number) 
      
      estimated_slope  = linear_coff[1]
      estimated_intercept = linear_coff[2]
      rem justify estimated parameters
      
      
      rem subroutine to get velocity and controller output array
                  
      vel_controloutput_array(Sinusfrequenz_Vibro,WegInd_RegOut, local_c,controlleroutput,vel_local,velocity_data,local_vel_pointer, data_sizetrain,WegInd_Digits,fulldata_number)
      rem fitted velocity from linear model
      '      if ((virboswitch_flag = 0) and (virboswitch_flag_1 = 0)) then
      Fit_vel_calcul(linear_coff,controlstart_step,fittedlinear_coff,fitted_vel,WegInd_RegOut )
      '    endif
      
      
     
      rem steps to trace back and obtain predicted distance
      
    
      MAX_correlationstep = total_shift
             
      rem calculate predicted distance
      if (Sinusfrequenz_Vibro >=0.13) then
        dec(local_c)
      endif
      predict_dist = predict_distance(linear_coff,estimated_slope_array,estimated_intercept_array,estimated_parameter_pointer,predict_distance_pointer,predict_full,estima_vel_sub,WegInd_Digits,windowsize,NEWpointer_average,local_c,distance_data, controlleroutput, data_sizetrain,MAX_correlationstep)
       
      rem record predict distance into array
      if (pre_dist_p <= data_sizetrain) then
        predict_distance_array[pre_dist_p] = predict_dist
        inc(pre_dist_p)
      else
        pre_dist_p =1 
      endif
     
      rem  subroutine for dead zone control and safety concern for controller output                    
      Control_output_final_regulation(ZERO_OFFSET,MAX_OUTPUT,Kp_Position,WegInd_Delta,Soll_Position_Regelungswert,time_delay_change_freq_2,time_delay_pointer_2,Switch_freq_depth_static,time_delay_change_freq_1,time_delay_pointer_1,Enable_vibro_mode_3,time_delay_change_freq,Abtastfrequenz,time_delay_pointer,Enable_vibro_mode_2,switch_frequency_button,Switch_freq_depth_1,Switch_freq_depth,stopbutton,DIO_Zylinder, pointer_average,PISTON_OFFSET,distance_data,windowsize,estimatedWegInd_RegOut,WegInd_RegOut,WegInd_Digits,vibration_amplitude_modelcontrol) 
        
     
     
      ' Pressure control with PI controller   
      '----------------------------------------------------------------------------------------------------------------
      
      PI_controller_porepressure(PaProMin,DIO_SpindelA,Pore_Control_Button,ZERO_OFFSET,Kp_Porepress,Ki_Porepress,PorePress_RegOut,PorePress_DeltaSum,LinearUpDown_Button,Linear_Pore_GO,Set_Soll_PorePressure,PorePress_Delta, Soll_Porepressure,PorePress_Digits)  
      
      PI_controller_sidepressure(actual_sidevolume,initial_cone_position,initial_strain,start_penetrat_depth,Circumf_Control_On,pi,r_0,PaProMin,Linear_Side_GO,Compass_OFF,Circumference_ON_Flag,Abschaltung_Reglungs_Wert,lowestWeg, WegInd_Digits,Additionalside_pressure_array,New_BC5_MODEL,Side_Addtion_Phys,Side_Addtion_Digits,Huang_su_formel,Huang_Su_par_1,Huang_Su_par_2,Huang_Su_par_3,Huang_Su_par_4,Huang_Su_par_5,Huang_Su_par_6,LVDT_Max, LVDT_1,LVDT_2,LVDT_3,BC5_MODEL_selection,linear_calibration_flag,DIO_SpindelB,Side_Control_Button,ZERO_OFFSET,Kp_Sidepress,Ki_Sidepress,SidePress_RegOut,SidePress_DeltaSum,LinearUpDown_Button,Linear_Pore_GO,Set_Soll_SidePressure,SidePress_Delta, Soll_Sidepressure,SidePress_Digits)  
 
      PI_controller_Axialpressure(Pre_Shearing_Button,Model_PreShearing,Amplitude_PreSh,Sinusfrequenz_preSh,PreSh_internal_counter,Linear_Axial_GO,PaProMin,DIO_SpindelC,Axial_Control_Button,ZERO_OFFSET,Kp_Axialpress,Ki_Axialpress,AxialPress_RegOut,AxialPress_DeltaSum,LinearUpDown_Button,Linear_Pore_GO,Set_Soll_AxialPressure,AxialPress_Delta, Soll_Axialpressure,AxialPress_Digits)  
   
      rem ////////////////////////////////////////////////////////////////////////////  
      rem model/based controller with simple linear regression model, implemented by zhengyu chen,
      rem latest modified date> 2022.06.08    \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ 
      

    Case MODUS_MAIN_CONTROL 'all Vibro/Spindel/Pressure Control 
      
      Digout(0,1)       'Freigabe fÃ¼r Moogventil
      Digout(1,1)       'Freigabe fÃ¼r Lasersensoren


      If ((Start_Test_Button = 1) and(Sinusfrequenz_Vibro*5000 > Vibro_internal_counter)) then 
        INC Vibro_internal_counter
      Else
        Vibro_internal_counter = 0
      EndIf
      
      Model_Vibro = Amplitude_Vibro * sin(2*3.14159/(Sinusfrequenz_Vibro*5000) * Vibro_internal_counter) 

      If (Start_Test_Button = 1) Then Soll_Position_Regelungswert = Soll_Position_Regelungswert - Vorschub 'activate vorschub
      If ((Start_Test_Button = 1) and (Vibro_Test_Button = 1)) Then 
        WegInd_Delta = WegInd_Digits - Soll_Position_Regelungswert + Model_Vibro 'Model activated
      Else  
        WegInd_Delta = WegInd_Digits - Soll_Position_Regelungswert 'Position halten1
      EndIf
                   
      If (WegInd_Digits < 16144) then 'EndE des Reglers nahe dem Ende
        Kp_Position_Vibro = 1
        Ki_Position_Vibro = 1
      EndIf   
          
      WegInd_DeltaSum  = WegInd_Delta + WegInd_DeltaSum

      If (WegInd_DeltaSum < -MAX_OUTPUT) Then WegInd_DeltaSum = -MAX_OUTPUT
      If (WegInd_DeltaSum > MAX_POS_OUTPUT) Then WegInd_DeltaSum = MAX_POS_OUTPUT

      If (Start_Test_Button = 1) Then
        WegInd_RegOut = ( Kp_Position_Vibro * WegInd_Delta ) + ( Ki_Position_Vibro * Taktzeit * WegInd_DeltaSum )
      Else
        WegInd_RegOut = (Kp_Position * WegInd_Delta)
      EndIf
      rem controller output for distance sensor data
      WegInd_RegOut = WegInd_RegOut + ZERO_OFFSET
      
      if (stopbutton = 0 ) then
        DAC(DIO_Zylinder,WegInd_RegOut)
      else
        DAC(DIO_Zylinder,32768)
      endif

      '-----------------------------------------------------------------------------------------------------------------------------------------------------

      If ((LinearUpDown_Button = 1) and (Linear_Pore_GO = 1))  Then 
        Soll_Porepressure  = Soll_Porepressure  + PaProMin
        Set_Soll_PorePressure =  Soll_Porepressure 
      EndIf
      
      If ((LinearUpDown_Button = 0) and (Linear_Pore_GO = 1))  Then
        Soll_Porepressure  = Soll_Porepressure  - PaProMin
        Set_Soll_PorePressure =  Soll_Porepressure  
      EndIf
      
      If (Linear_Pore_GO = 0) Then  Soll_Porepressure = Set_Soll_PorePressure 
      
      PorePress_Delta = Soll_Porepressure - PorePress_Digits
          
      PorePress_DeltaSum = pump_delta_sum(PorePress_DeltaSum,PorePress_Delta,Ki_Porepress)
      
      PorePress_RegOut = (Kp_Porepress * PorePress_Delta) + ( Ki_Porepress * PorePress_DeltaSum)
      PorePress_RegOut = PorePress_RegOut + ZERO_OFFSET
      
      'FPar_71 = PorePress_RegOut
              
      If (Pore_Control_Button = 0) Then              
        DAC(DIO_SpindelA, ZERO_OFFSET)
        Soll_Porepressure =  PorePress_Digits
        PorePress_DeltaSum = 0
      Else       
        DAC(DIO_SpindelA,PorePress_RegOut)
                  
      EndIf
      'FPar_71 =Soll_Porepressure - PorePress_Digits
      
      
      
      
      '-----------------------------------------------------------------------------------------------------------------------------------------------------------
      
      rem detect change in side volume 
      '      if ((LinearUpDown_Button = 1) and (Linear_Side_GO = 1))  Then 
      '        if (side_volume_pointer = 1) then
      '          rem record initial digital side volume 
      '          initial_side_volume = SpindelB_Digits
      '        endif
      '        inc(side_volume_pointer)
      '        rem actual side volume change in ml
      '        actual_side_volume_change = 0.03107*(SpindelB_Digits-initial_side_volume)
      '        rem if side volume change exceeds set value, stop loading
      '        if (0.03107*(SpindelB_Digits-initial_side_volume) >= side_volume_change) then Side_Control_Button = 0
      '      endif
    
    
      If ((LinearUpDown_Button = 1) and (Linear_Side_GO = 1))  Then 
        Soll_SidePressure  = Soll_SidePressure  + PaProMin
        Set_Soll_SidePressure = Soll_SidePressure
      EndIf
      If ((LinearUpDown_Button = 0) and (Linear_Side_GO = 1))  Then 
        Soll_SidePressure  = Soll_SidePressure  - PaProMin
        Set_Soll_SidePressure = Soll_SidePressure 
      EndIf
      If (Linear_Side_GO = 0) Then Soll_Sidepressure = Set_Soll_SidePressure 
      
     
      
       

      
      If ((Circumf_Control_On = 1) and (Start_Test_Button = 1)) then
        
       
        
        rem  If ((Circumf_Control_On = 1) ) then
        rem if (linear_calibration_flag = 0) then
        LVDT_Max = LVDT_1                               'max of all LVDT
        If (LVDT_2 > LVDT_Max) Then LVDT_Max = LVDT_2                        
        If (LVDT_3 > LVDT_Max) Then LVDT_Max = LVDT_3 
                        
        'Huang Su Formel 
        Side_Addtion_Phys = Huang_Su_par_1 * (LVDT_MAX)^6 + Huang_Su_par_2 * (LVDT_MAX)^5 + Huang_Su_par_3 * (LVDT_MAX)^4 + Huang_Su_par_4 * (LVDT_MAX)^3 + Huang_Su_par_5 * (LVDT_MAX)^2 + Huang_Su_par_6 * (LVDT_MAX) + Huang_Su_par_7
        Side_Addtion_Digits = Side_Addtion_Phys * 5.461333333  
                              
       
        rem apply new BC5 Model  
        New_BC5_model_additionside_pressure(initial_cone_position,initial_strain,start_penetrat_depth,pi,r_0,Additionalside_pressure_array,local_counter,Side_Addtion_Digits,Side_Addtion_Phys,actual_sidevolume,strain_linear_slope, delta_r, strain_linear_intercept,Actual_volume,initial_volume,linear_calibration_flag,lowestWeg,WegInd_Digits,localtimestep,weginitial_depth,initial_piston_position)  
  
      
        
      
      endif
       
      
      
         
      If (Circumf_Control_On = 1) Then 
        rem if we use calibration of new BC5, during the calibration stage, we don't apply 
        rem additional side pressure, after penetration depth reaches 100 mm, we start to apply 
        rem side pressure
         
        if (linear_calibration_flag = 1) then
          if (  lowestWeg <= WegInd_Digits) then 
            SidePress_Delta  = Soll_Sidepressure  - SidePress_Digits   
          else
            SidePress_Delta  = Soll_Sidepressure + Side_Addtion_Digits - SidePress_Digits
          endif
          rem if we switch to original calibration with Huangsu formula
        else
          SidePress_Delta  = Soll_Sidepressure + Side_Addtion_Digits - SidePress_Digits
        endif        
        Abschaltung_Reglungs_Wert = Soll_Sidepressure + Side_Addtion_Digits         
      Else    
      
        SidePress_Delta = Soll_Sidepressure - SidePress_Digits
        
        If ((Compass_OFF = 1) and (Circumference_ON_Flag = 1)) Then 
          SidePress_Delta = Abschaltung_Reglungs_Wert - SidePress_Digits
        EndIf  
      EndIf
     
     
      SidePress_DeltaSum = pump_delta_sum(SidePress_DeltaSum,SidePress_Delta,Ki_Sidepress)
       

      SidePress_RegOut = ( Kp_Sidepress * SidePress_Delta ) + ( Ki_Sidepress * SidePress_DeltaSum)
      SidePress_RegOut = SidePress_RegOut + ZERO_OFFSET                                                
                  
      If (Side_Control_Button = 0) Then              
        DAC(DIO_SpindelB, ZERO_OFFSET)
        Soll_Sidepressure =  SidePress_Digits
        SidePress_DeltaSum = 0
        
        
        
        
        
      Else       
       
        DAC(DIO_SpindelB,SidePress_RegOut)             
      EndIf
   
        
        
        
      '------------------------------------------------------------------------------------------------------------------------------------------------------------------

      If ((LinearUpDown_Button = 1) and (Linear_Axial_GO = 1)) Then
        Soll_AxialPressure = Soll_AxialPressure + PaProMin
        Set_Soll_AxialPressure  =  Soll_AxialPressure
      EndIf
      If ((LinearUpDown_Button = 0) and (Linear_Axial_GO = 1)) Then 
        Soll_AxialPressure = Soll_AxialPressure - PaProMin
        Set_Soll_AxialPressure  =  Soll_AxialPressure
      EndIf
      If (Linear_Axial_GO = 0) Then Soll_Axialpressure = Set_Soll_AxialPressure 
      
      If (Sinusfrequenz_preSh*5000 > PreSh_internal_counter) Then
        Inc PreSh_internal_counter
      Else
        PreSh_internal_counter = 0
      Endif
  
      Model_PreShearing = Amplitude_PreSh * sin(2*3.14159/(Sinusfrequenz_preSh*5000) * PreSh_internal_counter)'Amplitude Pre-Shearing 
  
      if (Pre_Shearing_Button = 1) Then AxialPress_Delta = Soll_Axialpressure + Model_PreShearing - AxialPress_Digits 
      if (Pre_Shearing_Button = 0) Then AxialPress_Delta = Soll_Axialpressure - AxialPress_Digits  
                                          
      PreShearingOutput =  (Soll_Axialpressure + Model_PreShearing)           

      AxialPress_DeltaSum = pump_delta_sum(AxialPress_DeltaSum,AxialPress_Delta,Ki_Axialpress)
      
      AxialPress_RegOut = ( Kp_Axialpress * AxialPress_Delta ) + ( Ki_Axialpress * AxialPress_DeltaSum)
      AxialPress_RegOut = AxialPress_RegOut + ZERO_OFFSET
      
      If (Axial_Control_Button = 0) Then              
        DAC(DIO_SpindelC, ZERO_OFFSET)
        Soll_Axialpressure =  AxialPress_Digits
        AxialPress_DeltaSum = 0
      Else       
        DAC(DIO_SpindelC,AxialPress_RegOut)             
      EndIf
      
      
      
    case MODUS_SHUTDOWN   '---------------------------------------------------------------------------------------------------------------------------------------------
      
      Digout(0,1)       'Freigabe fÃ¼r Moogventil
      Digout(1,1)       'Freigabe fÃ¼r Lasersensoren
                 
      If (Einfahren_Piston = 0) Then
     
        WegInd_Delta     = WegInd_Digits - Soll_Position_Regelungswert 
        WegInd_DeltaSum  = WegInd_Delta + WegInd_DeltaSum

        If (WegInd_DeltaSum < -MAX_OUTPUT) Then WegInd_DeltaSum = -MAX_OUTPUT
        If (WegInd_DeltaSum > MAX_POS_OUTPUT) Then WegInd_DeltaSum = MAX_POS_OUTPUT

        WegInd_RegOut = ( Kp_Position * WegInd_Delta )
  
        If (WegInd_RegOut< -MAX_OUTPUT) Then WegInd_RegOut= -MAX_OUTPUT
        If (WegInd_RegOut>  MAX_OUTPUT) Then WegInd_RegOut=  MAX_OUTPUT
        
        WegInd_RegOut= WegInd_RegOut + ZERO_OFFSET      
        DAC(DIO_Zylinder,WegInd_RegOut)  
      Else 
        DAC(DIO_Zylinder,31000)  
      Endif 
                 
    case MODUS_EMERGENCY '------------------------------------------------------------------------------------------------------------------------------------------ 
      
      Digout(0,1)       'Freigabe fÃ¼r Moogventil
      Digout(1,1)       'Freigabe fÃ¼r Lasersensoren           
      
      DAC(DIO_Zylinder, PISTON_OFFSET) 
      DAC(DIO_SpindelA, ZERO_OFFSET)   'SpindelA not moving
      DAC(DIO_SpindelB, ZERO_OFFSET)   'SpindelB not moving
      DAC(DIO_SpindelC, ZERO_OFFSET)   'SpindelC not moving

  EndSelect
  
  
  
  'From here reading out the RS232 Stream for the Cone Resistance and Tip Pressure
  val = read_fifo(1)
  If(val > -1) Then
    cnt = 0
    do
      cnt = cnt + 1
      FIFO_RS_RAW = val
      val = read_fifo(1)
    until (val = -1)
  Endif
  
  fifoused_ = FIFO_Full(1)
  
  If (synced_ = 0) Then
    If (fifoused_ > 200) Then
      z = 1
      j = 1
      
      do
        '2-byte Startsequenz (0xAA 0x55) aus dem FIFO suchen -  syn erst erfolgreich wenn Startsequenz 2x gefunden wurde
        INC j
        If(FIFO_RS_RAW = 0AAh) Then
          If(FIFO_RS_RAW = 055h) Then
            lastSeqNo = FIFO_RS_RAW
            For z = 1 to 35
              dummy = FIFO_RS_RAW
            next z
            INC synced_ 
          Endif
        Endif    
      until (synced_ = 2) or j >= 200
    Endif
  Endif
  
  'wenn Synchronisierung erfolgreich, das Daten entsprechend Protokoll auslesen
  If (synced_ >=2) Then
    If (fifoused_ > 38) Then
      ' kompletten Datensatz einlesen
      For z = 1 to 38
        FIFO_RS_SYNCED[z] = FIFO_RS_RAW
      next z
      
      'ersten beiden Bytes sind der header
      sync1 = FIFO_RS_SYNCED[1]
      sync2 = FIFO_RS_SYNCED[2]
      checksum = sync1 + sync2
      
      'prÃ¼fen ob der Datenstrom richtig synchronisiert ist
      If (checksum <> 00FFh) Then
        resync_ = resync_ + 1
        synced_ = 0
        dataValid_ = 0 
      Endif
      
      'type
      dummy = 0
      dummy = FIFO_RS_SYNCED[2 * 1 + 2]
      dummy = shift_left(dummy, 8)
      dummy = dummy + FIFO_RS_SYNCED[2*1 +1]
      dataValues[1] = dummy
      avisaroType = dummy
      
      'length
      dummy = 0
      dummy = FIFO_RS_SYNCED[2 * 2 + 2]
      dummy = shift_left(dummy, 8)
      dummy = dummy + FIFO_RS_SYNCED[2*2 +1]
      dataValues[2] = dummy

      'sequence No:
      dummy = 0
      dummy = FIFO_RS_SYNCED[2 * 3 + 2]
      dummy = shift_left(dummy, 8)
      dummy = dummy + FIFO_RS_SYNCED[2*3 +1]
      dataValues[3] = dummy

      '3xacceleration, 3x gyro (=6 Werte)
      'avisaro 16Bit Vorzeichenbehaftet -32768 ... 32768
      For z = 4 to 9
        dummy = 0
        dummy = FIFO_RS_SYNCED[2*z+2]
        dummy = shift_left(dummy, 8)
        dummy = dummy + FIFO_RS_SYNCED[2*z+1]
        
        If((dummy AND 8000h)=8000h) Then
          dummy = 0FFFF0000h or dummy
        Endif
        datavalues[z]  = dummy
      next z
      
      For z = 5 to 8
        dummy = 0
        dummy = FIFO_RS_SYNCED[4*z+4]
        dummy = shift_left(dummy, 8)
        dummy = dummy + FIFO_RS_SYNCED[4*z+3]
        dummy = shift_left(dummy, 8)
        dummy = dummy + FIFO_RS_SYNCED[4*z+2]
          
        'zahl im 2er Komplement. Wenn bit 24 (=Vorzeichen im MSB) in Avisaro-24bit-Zahl) gesetzt ist auch in der ADwin 32bit Zahl setzen.
        If ((dummy AND 800000h) = 800000h) Then 
          dummy = 0FF000000h or dummy
        Else
          dummy = 000FFFFFFh AND dummy
        Endif
        dataValues[5 + z] = dummy
        'SpitzEndruck
        If (z = 6) Then
          SpitzEndruck_D = dummy
        endif
        If (z = 7) Then
          Porendruck_D = dummy
        endif
      next z
        
      For z = 18 to 18
        dummy = 0
        dummy = FIFO_RS_SYNCED[2 * z + 2]
        dummy = shift_left(dummy, 8)
        dummy = dummy + FIFO_RS_SYNCED[2 * z + 1]
        dataValues[14] = dummy
      next z            
    endif
  endif
  
  
  
  
  FIFO_correlation = local_acceleration
  FIFO_correlation = (-420/33076) *vel_deviation
  FIFO_correlation = 1
  rem check first appearance of event that Programm_counter reaches 32768 
  rem modified by Zhengyu Chen 2022-05-10
  rem as for the case of data overflow (spikes appear). The value of program counter suddenly drops to 32768
  rem this is discovered by data analysis for datasets containing spikes.
  if (Programm_counter = 32768) then
    inc(BUG_flag)
    Par_12 = BUG_flag 
  endif

  rem for the case that program counter suddenly drops, spike bug will happen because of data overflow,
  rem then we don't write data
  rem modified by zhengyu chen, 2022.06.27
 
  'Writing all Values into the FIFO for communicating with Labview. Enables fast data saving
  rem modified by Zhengyu Chen 2022-04-29
  rem check if there are sufficient number of elements in FIFO to write into
  rem we only transmit fifo data into PC if above condition holds true
  program_counter_interval = programm_cycle[2] - programm_cycle[1]
  
  
  rem first check if there is dataoverflow (spike) bug
    
  if ( program_counter_interval =1 )THEN
    if (BUG_flag <= 1) then
      if (FIFO_Empty(50) > 50)  Then
        FIFO_MAIN_OUT = Programm_counter                                                ' 1 Cycles of Adwin program
        FIFO_MAIN_OUT = ((0.85068) * KraftHTM_Digits)-27600                             ' 2 Kraft in N
        FIFO_MAIN_OUT = (-420/33076) * WegInd_Digits+623                                ' 3 Weg Induktiv in mm
        rem digit values for pore, side and axial pressure are not for processing in Labview, don't read via FIFO.  
        FIFO_MAIN_OUT = PorePress_RegOut                                                ' 4 pore pressure in Digits 
        FIFO_MAIN_OUT = SidePress_RegOut                                                ' 5 Side pressure in Digits
        FIFO_MAIN_OUT = AxialPress_RegOut                                               ' 6 Axial pressure in Digits 
           
        FIFO_MAIN_OUT = LVDT_1                                                          ' 7 LVDT 1 in mm
        FIFO_MAIN_OUT = LVDT_2                                                          ' 8 LVDT 2 in mm
        FIFO_MAIN_OUT = LVDT_3                                                          ' 9 LVDT 3 in mm
        FIFO_MAIN_OUT = ((12000 / 65536) * (PorePress_Digits)) - 6000                   ' 10 Pore  Press in kPa
        FIFO_MAIN_OUT = ((12000 / 65536) * (SidePress_Digits)) - 6000                   ' 11 Side  Press in kPa
        FIFO_MAIN_OUT = ((12000 / 65536) * (AxialPress_Digits))- 6000                   ' 12 Axial Press in kPa     
        FIFO_MAIN_OUT = (0.0311 * SpindelA_Digits) - 1014.75                            ' 13 SpindelA in ml
        FIFO_MAIN_OUT = (0.03107* SpindelB_Digits) - 1015.90                            ' 14 SpindelB in ml
        FIFO_MAIN_OUT = (0.0312 * SpindelC_Digits) - 1019.53                            ' 15 SpindelC in ml
        FIFO_MAIN_OUT = Soll_Porepressure                                               ' 16 Soll Pore Pressure in Digits
        FIFO_MAIN_OUT = Soll_Sidepressure                                               ' 17 Soll Side Pressure in Digits
        FIFO_MAIN_OUT = Soll_Axialpressure                                              ' 18 Soll Axial Pressure in Digits
        FIFO_MAIN_OUT = LVDT_MAX                                                        ' 19 LVDT Max
        FIFO_MAIN_OUT = 0.5*((-420/33076) *desired_dist + 623)                                ' 20 desired distance in mm
        FIFO_MAIN_OUT = (-420/33076) *predict_dist + 623                                                   ' 21 predict distance in mm
        FIFO_MAIN_OUT = fitted_vel                                                      ' 22 fitted velocity from linear regression
     
          
        FIFO_MAIN_OUT = Valve_IST                                                       ' 23 Huang_Su Addition in kPa
        FIFO_MAIN_OUT = Spitzendruck_D                                                  ' 24 Tip Resistance in Digits       
        FIFO_MAIN_OUT = Porendruck_D                                                    ' 25 Pore Pressure in Digits
        FIFO_MAIN_OUT = WegInd_RegOut                                                   ' 26 PI- Control OUT in Digits
          
        FIFO_MAIN_OUT = vel_local                                                      ' 27 local velocity
          
        FIFO_MAIN_OUT = estimatedWegInd_RegOut                                          ' 28 estimated controller output
        FIFO_MAIN_OUT = normalize_vel                                                   ' 29 normalized velocity data   '
        FIFO_MAIN_OUT = normalize_control                                               ' 30 normalized controller output
  
          
        FIFO_MAIN_OUT = delta_r                                                         ' 31 strain in mm
        FIFO_MAIN_OUT = Side_Addtion_Phys                                               ' 32 additional side pressure 
        
        FIFO_MAIN_OUT = weginitial_depth                                                ' 33 initial position of piston
        FIFO_MAIN_OUT = WegInd_Digits                                                   '34 current position                  
        FIFO_MAIN_OUT = initial_volume                                                  '35   initial volume
        FIFO_MAIN_OUT = Actual_volume                                                   '36 actual volume
        FIFO_MAIN_OUT = actual_sidevolume                                               '37 actual side volume
         
        
            
       
      else
        rem clear out fifo data if fifo empty space is too small
        rem modified by zhengyu chen 2022-05-10
        fifo_clear(50)
      endif
    else
      rem record the case that dataoverflow happens
      rem modified by zhengyu chen 2022-05-02
      rem first start the labview CPT teststand project, and then we could complie ADbasic code
      rem otherwise PAR_12 WILL TRUN to be 1 immediately, because FIFO data is not sending to PC and thus we will 
      rem run out of fifo buffer immediately.
   
      rem clear out fifo data if bug happens
      rem modified by zhengyu chen 2022-05-10
      fifo_clear(50)
    endif
  endif
 
  rem when we detect error, replace current erroneuos progamm counter value by the value at last time step
  rem this is used for the case system is wrong and thus program counter has many wrong values around 32768, 
  rem we need to make sure last recorded progamm counter has correct value
  
  if (Programm_counter > 2) then
    if (programm_cycle[2] <= programm_cycle[1] )THEN
      programm_cycle[2] = programm_cycle[1]
      
    endif
  endif
  rem modified By zhengyu chen 2022.04.26
  
  rem record processing clock cycles at the point which data are written into Fifo
  fifotime = Read_Timer()

  rem time resolution system clock of T12.1 processor is 1.5ns
  rem calculate real time from clock cycles, unit: ns
  

  rem modified By zhengyu chen 2022.04.26
