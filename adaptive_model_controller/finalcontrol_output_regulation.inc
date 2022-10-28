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
Sub  Control_output_final_regulation(stopbutton,DIO_Zylinder, pointer_average,PISTON_OFFSET,distance_data[],windowsize,estimatedWegInd_RegOut,WegInd_RegOut,WegInd_Digits,vibration_amplitude_modelcontrol) as float
                         
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
    if (pointer_average = 1) then

      if (ABSF((-420/33076) *(distance_data[windowsize] - WegInd_Digits)) > 0.5) then
        rem if controller performance is not good, switch to PI controller
        DAC(DIO_Zylinder,PISTON_OFFSET)
        stopbutton = 1
      endif
     
    else
                                                      
                                                  
      if (ABSF((-420/33076) *(distance_data[pointer_average-1] - WegInd_Digits)) > 0.5) then
        rem if controller performance is not good, switch to PI controller
        DAC(DIO_Zylinder,PISTON_OFFSET)
        stopbutton = 1
      else
        DAC(DIO_Zylinder,WegInd_RegOut)
      endif
    endif  
                                                
    rem check if we touches the lower bound of piston movement
    if ((-420/33076) * WegInd_Digits+623 <= 420 )then 
                                                    
      DAC(DIO_Zylinder,WegInd_RegOut)
    else
      DAC(DIO_Zylinder,PISTON_OFFSET)
                                                      
    endif
  else
    DAC(DIO_Zylinder,PISTON_OFFSET)      
  endif                           

         
                     
                 
                   
                   
      
      
              
 
  
  
  
endsub
