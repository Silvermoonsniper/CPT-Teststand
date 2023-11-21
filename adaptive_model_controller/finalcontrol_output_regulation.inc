
rem  subroutine for dead zone control and safety concern for controller output 

rem input args:
rem        stopbutton: stop button
rem        DIO_Zylinder: piston name
rem        pointer_average: pointer for distance data array
rem        PISTON_OFFSET: piston still offset (slighly up)
rem        distance_data[]: distance data array
rem        windowsize: size for distance data array
rem        estimatedWegInd_RegOut: estimated controller output from model
rem        WegInd_RegOut: actual controller output
rem        WegInd_Digits: distance sensor data
rem        vibration_amplitude_modelcontrol: set vibration amplitude 
rem        ZERO_OFFSET: 32768,digital output which corresponds to 0v, 
rem        MAX_OUTPUT: maximum control output for piston movement in manuel mode
rem        Kp_Position: porportional constant of PI controller for piston moevement
rem        WegInd_Delta: error in PI controller 
rem        Soll_Position_Regelungswert:
rem        time_delay_change_freq_2: time delay for static peneration mode
rem        time_delay_pointer_2: pointer for time delay
rem        Switch_freq_depth_static: depth that switches to static penetration
rem        time_delay_change_freq_1,: time delay for second vibro mode
rem        time_delay_pointer_1: pointer for time delay, third vibro mode
rem        Enable_vibro_mode_3: button to enable third vibro mode
rem        time_delay_change_freq: time delay for second vibro mode
rem        Abtastfrequenz: sampling frequency
rem        time_delay_pointer: pointer for time delay, second vibro mode
rem        Enable_vibro_mode_2: button to enable second vibro mode
rem        switch_frequency_button: button to enable switch vibro mode
rem        Switch_freq_depth_1: depth to switch to third vibro mode
rem        Switch_freq_depth: depth to switch to second vibro mode

Sub  Control_output_final_regulation(ZERO_OFFSET,MAX_OUTPUT,Kp_Position,WegInd_Delta,Soll_Position_Regelungswert,time_delay_change_freq_2,time_delay_pointer_2,Switch_freq_depth_static,time_delay_change_freq_1,time_delay_pointer_1,Enable_vibro_mode_3,time_delay_change_freq,Abtastfrequenz,time_delay_pointer,Enable_vibro_mode_2,switch_frequency_button,Switch_freq_depth_1,Switch_freq_depth,stopbutton,DIO_Zylinder, pointer_average,PISTON_OFFSET,distance_data[],windowsize,estimatedWegInd_RegOut,WegInd_RegOut,WegInd_Digits,vibration_amplitude_modelcontrol) as float
                         
  if (estimatedWegInd_RegOut > 200000) then
    estimatedWegInd_RegOut = 200000
  endif 
  if (estimatedWegInd_RegOut < -200000) then
    estimatedWegInd_RegOut = -200000
  endif
                             
  if (WegInd_RegOut > 200000) then
    WegInd_RegOut = 200000
  endif 
  if (WegInd_RegOut < -200000) then
    WegInd_RegOut = -200000
  endif
                           
  
  if (switch_frequency_button = 1) then
      
    rem second vibro mode, first hold the position for given time delay
   
    if (Enable_vibro_mode_2 = 1) then  
      
      if ((-420/33076) * WegInd_Digits+623   >= Switch_freq_depth ) then
           
        if (time_delay_pointer < round(time_delay_change_freq*Abtastfrequenz)) then
          
          rem stop penetration first
          Soll_Position_Regelungswert = (Switch_freq_depth-623)* (-33076/420)  'Übergabe an den nächsten Case
      
          rem keep position of piston
          
          WegInd_Delta = WegInd_Digits - Soll_Position_Regelungswert 'auf letzte Position geregelt
          WegInd_RegOut = (Kp_Position * WegInd_Delta)
        
          If (WegInd_RegOut < -MAX_OUTPUT)  Then WegInd_RegOut = -MAX_OUTPUT
          If (WegInd_RegOut > MAX_OUTPUT)  Then WegInd_RegOut = MAX_OUTPUT
  
          WegInd_RegOut= WegInd_RegOut + ZERO_OFFSET
          
        endif                     
      endif
    endif
    
    rem third vibro mode, first hold the position for given time delay
   
    if (Enable_vibro_mode_3 = 1) then
      if ((-420/33076) * WegInd_Digits+623   >= Switch_freq_depth_1 ) then
        
        
        if (time_delay_pointer_1 < round(time_delay_change_freq_1*Abtastfrequenz)) then
         
          rem stop penetration first
          Soll_Position_Regelungswert = (Switch_freq_depth_1-623)* (-33076/420)   'Übergabe an den nächsten Case
      
          rem keep position of piston
          
          WegInd_Delta = WegInd_Digits - Soll_Position_Regelungswert 'auf letzte Position geregelt
          WegInd_RegOut = (Kp_Position * WegInd_Delta)
        
          If (WegInd_RegOut < -MAX_OUTPUT)  Then WegInd_RegOut = -MAX_OUTPUT
          If (WegInd_RegOut > MAX_OUTPUT)  Then WegInd_RegOut = MAX_OUTPUT
  
          WegInd_RegOut= WegInd_RegOut + ZERO_OFFSET
          
        endif                     
        
      endif
    endif
    
    rem fourth mode: static penetration, first hold position of given time delay
    if (time_delay_change_freq_2 = 0) then time_delay_change_freq_2 = 0.001
    if  ((-420/33076) * WegInd_Digits+623   >= Switch_freq_depth_static )  then
      inc(time_delay_pointer_2)
      if (time_delay_pointer_2 < round(time_delay_change_freq_2*Abtastfrequenz))  then
        
       
        Soll_Position_Regelungswert = (Switch_freq_depth_static-623)* (-33076/420)   'Übergabe an den nächsten Case
      
        rem keep position of piston
          
        WegInd_Delta = WegInd_Digits - Soll_Position_Regelungswert 'auf letzte Position geregelt
        WegInd_RegOut = (Kp_Position * WegInd_Delta)
        
        If (WegInd_RegOut < -MAX_OUTPUT)  Then WegInd_RegOut = -MAX_OUTPUT
        If (WegInd_RegOut > MAX_OUTPUT)  Then WegInd_RegOut = MAX_OUTPUT
  
        WegInd_RegOut= WegInd_RegOut + ZERO_OFFSET
        
          
      else
        
        rem static CPT parameter, 20 mm/s penetration
        
       
        '
        Vorschub = (20*78.01)/5000
        
        Amplitude_Vibro = 0       
        
        Model_Vibro = Amplitude_Vibro * sin(2*3.14159/(Sinusfrequenz_Vibro*5000) * Vibro_internal_counter) 
        
                
        Soll_Position_Regelungswert = Soll_Position_Regelungswert - Vorschub 'activate vorschub
                
        WegInd_Delta = WegInd_Digits - Soll_Position_Regelungswert + Model_Vibro 'Model activated
                
                           
        If (WegInd_Digits < 16144) then 'EndE des Reglers nahe dem Ende
          Kp_Position_Vibro = 1
          Ki_Position_Vibro = 1
        EndIf   
                  
        WegInd_DeltaSum  = WegInd_Delta + WegInd_DeltaSum
        
        If (WegInd_DeltaSum < -MAX_OUTPUT) Then WegInd_DeltaSum = -MAX_OUTPUT
        If (WegInd_DeltaSum > MAX_POS_OUTPUT) Then WegInd_DeltaSum = MAX_POS_OUTPUT
        
               
        WegInd_RegOut = ( Kp_Position_Vibro * WegInd_Delta ) + ( Ki_Position_Vibro * Taktzeit * WegInd_DeltaSum )
               
        rem controller output for distance sensor data
        WegInd_RegOut = WegInd_RegOut + ZERO_OFFSET
        
      endif
    endif
  endif
    
                        
  rem output controller output
  rem check if controller stop button is pressed
     
              
  '      if (absf(((-420/33076) * (WegInd_Digits-desired_dist)) ) >= 0.3) then
  '                                                                                      
  '        DAC(DIO_Zylinder,32622) 
  '        stopbutton = 1
  '      endif
  if (stopbutton = 0 ) then
                     
                         
    rem when piston at the top region, move down for a certain distance for safety
                   
    if (((-420/33076) * WegInd_Digits+623)  <= 1*vibration_amplitude_modelcontrol) then
                    
      WegInd_RegOut = 34000
      DAC(DIO_Zylinder,WegInd_RegOut)
    
    endif
                                                
    rem check if measured distance deviates too much away from previous measured (0.5 mm for this case), if so, switch to PI controller for safety
    '    if (pointer_average = 1) then
    '    
    '      if (ABSF((-420/33076) *(distance_data[windowsize] - WegInd_Digits)) > 0.5) then
    '        rem if controller performance is not good, switch to PI controller
    '        DAC(DIO_Zylinder,PISTON_OFFSET)
    '        stopbutton = 1
    '      endif
    '         
    '    else
    '                                                          
    '                                                      
    '      if (ABSF((-420/33076) *(distance_data[pointer_average-1] - WegInd_Digits)) > 0.5) then
    '        rem if controller performance is not good, switch to PI controller
    '        DAC(DIO_Zylinder,PISTON_OFFSET)
    '        stopbutton = 1
    '      else
    '        DAC(DIO_Zylinder,WegInd_RegOut)
    '      endif
    '    endif  
                                                
    rem check if we touches the lower bound of piston movement
    if ((-420/33076) * WegInd_Digits+623 <= 420 )then 
                                                    
      DAC(DIO_Zylinder,WegInd_RegOut)
    else
      DAC(DIO_Zylinder,PISTON_OFFSET)
                                                      
    endif
  else
    
    Soll_Position_Regelungswert = WegInd_Digits   'Übergabe an den nächsten Case
      
    rem keep position of piston
          
    WegInd_Delta = WegInd_Digits - Soll_Position_Regelungswert 'auf letzte Position geregelt
    WegInd_RegOut = (Kp_Position * WegInd_Delta)
        
    If (WegInd_RegOut < -MAX_OUTPUT)  Then WegInd_RegOut = -MAX_OUTPUT
    If (WegInd_RegOut > MAX_OUTPUT)  Then WegInd_RegOut = MAX_OUTPUT
  
    WegInd_RegOut= WegInd_RegOut + ZERO_OFFSET
    DAC(DIO_Zylinder,WegInd_RegOut)  
   
  endif                           

         
                     
                 
                   
                   
      
      
              
 
  
  
  
endsub
