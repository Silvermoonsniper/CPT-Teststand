rem ########################################################################################
rem new BC5 model to calculate additional side pressure with volumetric strain
rem date:2022.08.23
rem Zhengyu Chen
rem pi: pi value
rem r_0: initial strain
rem Additionalside_pressure_array[]: addtional side pressure array
rem local_counter:counter for recoding additional side pressure
rem Side_Addtion_Digits: digital additional side pressure
rem Side_Addtion_Phys: additional side pressure in kpa
rem actual_sidevolume: actual side volume
rem strain_linear_slope: slope for linear new BC5 model
rem  delta_r: volumetric strain
rem  strain_linear_intercept: intercept for linear new BC5 model
rem Actual_volume: actual volume
rem initial_volume: initial volume
rem linear_calibration_flag: button to activate new BC5 Model
rem lowestWeg: starting point of penetration
rem WegInd_Digits: distance measurement
rem localtimestep: time step
rem weginitial_depth: initial position of pistion, set in labview
rem initial_piston_position: initial piston position when enters into this programm
Sub New_BC5_model_additionside_pressure(initial_cone_position,initial_strain,start_penetrat_depth,pi,r_0,Additionalside_pressure_array[],local_counter,Side_Addtion_Digits,Side_Addtion_Phys,actual_sidevolume,strain_linear_slope, delta_r, strain_linear_intercept,Actual_volume,initial_volume,linear_calibration_flag,lowestWeg,WegInd_Digits,localtimestep,weginitial_depth,initial_piston_position)      as float
  rem #############################
  rem calibration  for linear model
         
        
        
  rem record starting position of piston which is set in the labview
  if (localtimestep = 1) then 
                                            
    weginitial_depth = (initial_cone_position-623)*(33076/-420) 
          
    localtimestep = localtimestep + 1
  endif
        
        
  rem store initial measured position of piston
  if(p = 1) then
    initial_piston_position = WegInd_Digits
    
    inc(p)
  endif
         
  rem if we apply new BC5 model
  if (linear_calibration_flag = 1) then
    rem \\\\\\\\\\\\\\\\\ new calibration model
    rem calculate largest penetration depth in digitla value to apply BC5 linear MODEL
    rem unit mm     
    lowestWeg = (start_penetrat_depth -623)*(33076/-420)
        
    rem initial strain  
  
    r_0 = initial_strain
                    
   
           
    rem when we don't reach penetration point, which is set is labview, typically 100mm       
    if (lowestWeg <= WegInd_Digits) then 
      rem calculate initial volume in physical values
      initial_volume = absf(pi*r_0*r_0*((-420/33076) *(WegInd_Digits-weginitial_depth)))
      rem the actual volume is initial volume plus deformaton, during penetration the side volume is
      rem decreasing, 
      Actual_volume =  absf(1000*((0.03107* (actual_sidevolume )))) 
            

             
                        
      rem calculate change of strain
      if (initial_volume >0) then
        rem calculate strain ratio r/r0
        delta_r = Actual_volume/initial_volume
          
        
      ENDIF
        
        
                   
      rem calculate desired side pressure
      Side_Addtion_Phys = strain_linear_slope*delta_r + strain_linear_intercept
      rem calculate desired side pressure in digital values
      Side_Addtion_Digits = Side_Addtion_Phys * 5.461333333 
            

           
      rem store updating initial volume value
      if (local_counter < 2) then
        Additionalside_pressure_array[local_counter+1]= Side_Addtion_Digits
      else
        Additionalside_pressure_array[1] =  Additionalside_pressure_array[2]  
        Additionalside_pressure_array[2] =  Side_Addtion_Digits
      endif 
      local_counter = local_counter + 1
           
      rem after calibration stage, we apply the model
    else
          
      rem calculate initial volume in physical values
      initial_volume = absf(pi*r_0*r_0*((-420/33076) *(WegInd_Digits-weginitial_depth)))
         
      rem the actual volume is initial volume plus deformaton
      Actual_volume =   absf(1000*((0.03107* (actual_sidevolume )) )) 
            
            

            
            
      rem calculate change of strain
      if (initial_volume >0) then
        rem calculate strain ratio r/r0
        delta_r = Actual_volume/initial_volume
            
               
        
      ENDIF
      rem apply additional side pressure
      rem calculate desired side pressure
      Side_Addtion_Phys = strain_linear_slope*delta_r + strain_linear_intercept
      rem calculate desired side pressure in digital values
      Side_Addtion_Digits = Side_Addtion_Phys * 5.461333333    
          
         
            
    endif
    
        
      
  endif

  

endsub



