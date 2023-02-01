Function pump_delta_sum(Press_DeltaSum,Press_Delta,Ki) as float
  
  Press_DeltaSum  = Press_Delta + Press_DeltaSum
      
  If ((Press_Delta > 15)  and (Press_DeltaSum < 0)) Then  
    Press_DeltaSum = Press_DeltaSum * 0.9
  else
    If ((Press_Delta < -15) and (Press_DeltaSum > 0)) Then  
      Press_DeltaSum = Press_DeltaSum * 0.9
    else 
      If ((Press_Delta < 10)  and (Press_Delta > -10))   Then  Press_DeltaSum = Press_DeltaSum * 0.9995
  
    Endif
  Endif
  If ( Ki * Press_DeltaSum < -20000) Then Press_DeltaSum = -20000 / Ki
  If ( Ki * Press_DeltaSum > 20000)  Then Press_DeltaSum = 20000  / Ki
  
  pump_delta_sum = Press_DeltaSum
  
EndFunction



rem PI controller part for pressure control

Sub PI_controller_porepressure(PaProMin,DIO_SpindelA,Pore_Control_Button,ZERO_OFFSET,Kp_Porepress,Ki_Porepress,PorePress_RegOut,PorePress_DeltaSum,LinearUpDown_Button,Linear_Pore_GO,Set_Soll_PorePressure,PorePress_Delta, Soll_Porepressure,PorePress_Digits)  as float
  
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
  
endsub
'
'
sub  PI_controller_sidepressure(actual_sidevolume,initial_cone_position,initial_strain,start_penetrat_depth,Circumf_Control_On,pi,r_0,PaProMin,Linear_Side_GO,Compass_OFF,Circumference_ON_Flag,Abschaltung_Reglungs_Wert,lowestWeg, WegInd_Digits,Additionalside_pressure_array[],New_BC5_MODEL,Side_Addtion_Phys,Side_Addtion_Digits,Huang_su_formel,Huang_Su_par_1,Huang_Su_par_2,Huang_Su_par_3,Huang_Su_par_4,Huang_Su_par_5,Huang_Su_par_6,LVDT_Max, LVDT_1,LVDT_2,LVDT_3,BC5_MODEL_selection,linear_calibration_flag,DIO_SpindelB,Side_Control_Button,ZERO_OFFSET,Kp_Sidepress,Ki_Sidepress,SidePress_RegOut,SidePress_DeltaSum,LinearUpDown_Button,Linear_Pore_GO,Set_Soll_SidePressure,SidePress_Delta, Soll_Sidepressure,SidePress_Digits)  as float
  '-----------------------------------------------------------------------------------------------------------------------------------------------------------
      
  If ((LinearUpDown_Button = 1) and (Linear_Side_GO = 1))  Then 
    Soll_SidePressure  = Soll_SidePressure  + PaProMin
    Set_Soll_SidePressure = Soll_SidePressure
  EndIf
  If ((LinearUpDown_Button = 0) and (Linear_Side_GO = 1))  Then 
    Soll_SidePressure  = Soll_SidePressure  - PaProMin
    Set_Soll_SidePressure = Soll_SidePressure 
  EndIf
  If (Linear_Side_GO = 0) Then Soll_Sidepressure = Set_Soll_SidePressure 
      
     
      
      
       

      
 
  '        If (Circumf_Control_On = 1) then
         
  REM MODEL SELECTION
  if (linear_calibration_flag = 1) then
    BC5_MODEL_selection = 2
  else
    BC5_MODEL_selection = 0
  endif
  rem record LVDT data
  LVDT_Max = LVDT_1                               'max of all LVDT
  If (LVDT_2 > LVDT_Max) Then LVDT_Max = LVDT_2  
                               
  If (LVDT_3 > LVDT_Max) Then LVDT_Max = LVDT_3 
  rem use case structures to switch between different BC5 MODEL
  selectcase BC5_MODEL_selection
    case Huang_su_formel
          
              
      'Huang Su Formel 
      Side_Addtion_Phys = Huang_Su_par_1 * (LVDT_MAX)^6 + Huang_Su_par_2 * (LVDT_MAX)^5 + Huang_Su_par_3 * (LVDT_MAX)^4 + Huang_Su_par_4 * (LVDT_MAX)^3 + Huang_Su_par_5 * (LVDT_MAX)^2 + Huang_Su_par_6 * (LVDT_MAX) + Huang_Su_par_7
      Side_Addtion_Digits = Side_Addtion_Phys * 5.461333333  
                 
            
           
          
            
    case New_BC5_MODEL                   
            
            
      rem apply new BC5 Model
              
      New_BC5_model_additionside_pressure(initial_cone_position,initial_strain,start_penetrat_depth,pi,r_0,Additionalside_pressure_array,local_counter,Side_Addtion_Digits,Side_Addtion_Phys,actual_sidevolume,strain_linear_slope, delta_r, strain_linear_intercept,Actual_volume,initial_volume,linear_calibration_flag,lowestWeg,WegInd_Digits,localtimestep,weginitial_depth,initial_piston_position)  
   
       
  EndSelect
          
   
        
        
  rem if additional side pressure is applied
    
          
  
  rem if we use calibration of new BC5, during the calibration stage, we don't apply 
  rem additional side pressure, after penetration depth reaches 100 mm, we start to apply 
  rem side pressure
  rem if additional side pressure is  applied
  if (Circumf_Control_On = 1) then
    if (linear_calibration_flag = 1) then
      if (  lowestWeg <= WegInd_Digits) then 
        SidePress_Delta  = Soll_Sidepressure  - SidePress_Digits
                
                    
      else
        SidePress_Delta  = Soll_Sidepressure + Side_Addtion_Digits - SidePress_Digits
        
        
      endif
     
      rem with HUang su calibration   
    else
            
      rem calculate difference between desired pressure and measured one
                
      SidePress_Delta  = Soll_Sidepressure + Side_Addtion_Digits - SidePress_Digits
      
      
    endif
    Abschaltung_Reglungs_Wert = Soll_Sidepressure + Side_Addtion_Digits  
                     
          
    
    
  else      
    
    rem if additional side pressure is  not applied
    SidePress_Delta = Soll_Sidepressure - SidePress_Digits
          
    If ((Circumference_ON_Flag = 1)) Then 
      SidePress_Delta = Abschaltung_Reglungs_Wert - SidePress_Digits
    EndIf  
  endif
        
       
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
endsub







rem subroutine for PI controller for axial pressure
Sub PI_controller_Axialpressure(Pre_Shearing_Button,Model_PreShearing,Amplitude_PreSh,Sinusfrequenz_preSh,PreSh_internal_counter,Linear_Axial_GO,PaProMin,DIO_SpindelC,Axial_Control_Button,ZERO_OFFSET,Kp_Axialpress,Ki_Axialpress,AxialPress_RegOut,AxialPress_DeltaSum,LinearUpDown_Button,Linear_Pore_GO,Set_Soll_AxialPressure,AxialPress_Delta, Soll_Axialpressure,AxialPress_Digits)  as float
          
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
  
  
endsub
