rem parameter setup

Sub  parameter_setup(Switch_freq_depth_static,Enable_vibro_mode_2,Enable_vibro_mode_3,Switch_freq_depth_1,time_delay_pointer_1,time_delay_change_freq_1,time_delay_change_freq,damping_amplitude,penetration_vel,virboswitch_flag_1,virboswitch_flag,time_delay_pointer,Switch_freq_depth,intervalp,switch_frequency_button,linear_calibration_flag,WegInd_Digits ,controlstart_step, predict_dist,programm_cycle[],stopbutton,total_shift ,WegInd_RegOut,fulldata_number,Abtastfrequenz,original_vib_freq,PorePressOffset,AxialPressOffset,SidePressOffset,Sinusfrequenz_Vibro,Programm_counter)  as float
  rem update offset for pressure sensors
  PorePressOffset  = Fpar_20
  AxialPressOffset = Fpar_21  
  SidePressOffset  = Fpar_22
 
  rem update depth for switch vibro or static peneration modes
  Switch_freq_depth  =    Fpar_69
  Switch_freq_depth_1 =   Fpar_70
  Switch_freq_depth_static =  Fpar_46

  if (switch_frequency_button = 0 ) then
    original_vib_freq = Fpar_26
    damping_amplitude = fpar_23
    penetration_vel = fpar_24 
  
     
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
  
  rem specify background vel
  
              
        
  if (Sinusfrequenz_Vibro < 10) then intervalp = round(40*(10 - Sinusfrequenz_Vibro) + 200)
  if (Sinusfrequenz_Vibro >= 10) then intervalp = round(14*(10 - Sinusfrequenz_Vibro) + 200)
 
  rem set initial time delay
  total_shift = 125 + round(0.5*(intervalp- 200)) 
  par_45 = total_shift 
  rem model-based controller start button
  controlstart_step =  Par_39  
  
  rem set calibration button with Labview
  linear_calibration_flag = par_72

  
  
  '  --------------------------------------------------------------------------------------------------
  rem automatically change the frequency when we reach certain depth
  
  if (time_delay_change_freq = 0) then time_delay_change_freq = 0.001
  if (time_delay_change_freq_1 = 0) then time_delay_change_freq_1 = 0.001
  
  
  if (switch_frequency_button = 1) then
    
    rem second vibro mode
    if (Enable_vibro_mode_2 = 1) then
      if ((-420/33076) * WegInd_Digits+623   >= Switch_freq_depth ) then
        inc(time_delay_pointer)
                        
        if (time_delay_pointer >= round(time_delay_change_freq*Abtastfrequenz)) then
              
          rem set switch flag to 1
          virboswitch_flag = 1
         
          Sinusfrequenz_Vibro = fpar_26
          damping_amplitude = fpar_23
          penetration_vel = fpar_24                              
          rem set time delay
                            
          rem set initial time delay
          if (Sinusfrequenz_Vibro < 10) then intervalp = 40*(10 - Sinusfrequenz_Vibro) + 200
          if (Sinusfrequenz_Vibro >= 10) then intervalp = 14*(10 - Sinusfrequenz_Vibro) + 200
                  
          rem set initial time delay
          total_shift =    125 + round(0.5*(intervalp- 200))
          par_45 = total_shift
                               
                
              
           
        endif      
      endif
    endif
    rem  third vibro mode
    if (virboswitch_flag_1 = 1) then virboswitch_flag = 0
    
    
    if (Enable_vibro_mode_3 = 1) then 
      if ((-420/33076) * WegInd_Digits+623   >= Switch_freq_depth_1 ) then
        inc(time_delay_pointer_1)
                       
      
        if (time_delay_pointer_1 >= round(time_delay_change_freq_1*Abtastfrequenz)) then
          rem set switch flag to 1
          virboswitch_flag_1 = 1     
             
          Sinusfrequenz_Vibro = fpar_26
          damping_amplitude = fpar_23
          penetration_vel = fpar_24             
                                           
          rem set time delay
                            
          rem set initial time delay
          if (Sinusfrequenz_Vibro < 10) then intervalp = 40*(10 - Sinusfrequenz_Vibro) + 200
          if (Sinusfrequenz_Vibro >= 10) then intervalp = 14*(10 - Sinusfrequenz_Vibro) + 200
                  
          rem set initial time delay
          total_shift =   125 + round(0.5*(intervalp- 200))
          par_45 = total_shift
                               
       
           
        endif      
      endif
    endif
    
    
   
  endif
 
  
  if (Sinusfrequenz_Vibro  < 0.13) then
    
    Sinusfrequenz_Vibro = 0.13
  endif
endsub
