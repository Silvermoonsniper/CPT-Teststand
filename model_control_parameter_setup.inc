rem parameter setup
rem  intervalp: number of datapoints for velocity calculation
rem  switch_frequency_button: button to switch frequency during model control
rem  linear_calibration_flag: flag to switch on NEW BC5 for calibration chamber
rem  WegInd_Digits: measured digital distance data
rem  controlstart_step: button to switch on model control
rem  predict_dist: predicted distance from model
rem  programm_cycle[]: array to store progamm counnters
rem  stopbutton: stop button
rem  total_shift: data points for time delay
rem  WegInd_RegOut: control output for distance
rem  fulldata_number: number of datapoints for one period
rem  Abtastfrequenz: sampling frequency
rem  original_vib_freq: first set vibration frequency
rem  PorePressOffset: offset for pore pressure sensor
rem  AxialPressOffset: offset for axial pressure sensor
rem  SidePressOffset: offset for side pressure sensor
rem  Sinusfrequenz_Vibro: current vibration frequency
rem  Programm_counter: counter to record number of program cycles
Sub  parameter_steup(intervalp,switch_frequency_button,linear_calibration_flag,WegInd_Digits ,controlstart_step, predict_dist,programm_cycle[],stopbutton,total_shift ,WegInd_RegOut,fulldata_number,Abtastfrequenz,original_vib_freq,PorePressOffset,AxialPressOffset,SidePressOffset,Sinusfrequenz_Vibro,Programm_counter)  as float
  PorePressOffset  = Fpar_20
  AxialPressOffset = Fpar_21  
  SidePressOffset  = Fpar_22
 
  REM set vibration frequency  
  Sinusfrequenz_Vibro = Fpar_26

  if (switch_frequency_button = 0 ) then
    original_vib_freq = Fpar_26
   
  endif
  rem initialize number of points for 1 period
  fulldata_number  = 1*Abtastfrequenz/Sinusfrequenz_Vibro
  estimatedWegInd_RegOut =  WegInd_RegOut
  
  par_76 = fulldata_number 
  
  Inc Programm_counter 'Zeitstempel +1 
  stopbutton = Fpar_71
  rem store programm conter
  if (Programm_counter < 2) then
    programm_cycle[Programm_counter]= Programm_counter
  else
    programm_cycle[1] = programm_cycle[2]  
    programm_cycle[2] = Programm_counter
  endif
  

 
  
    
   
    
  rem initialize desired and predict distance 
  
  predict_dist =  WegInd_Digits 
  if (Sinusfrequenz_Vibro < 10) then intervalp = 40*(10 - Sinusfrequenz_Vibro) + 200
  if (Sinusfrequenz_Vibro >= 10) then intervalp = 14*(10 - Sinusfrequenz_Vibro) + 200
  
  rem set initial time delay
  total_shift = 125 + round(0.5*(intervalp- 200)) 
  par_45 = total_shift 
  rem model-based controller start button
  controlstart_step =  Par_39  
  
  rem set calibration button with Labview
  linear_calibration_flag = par_72

endsub

