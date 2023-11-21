
sub addition_triangular_vol(Volume_triangular,advance_depth_sample,inner_radius,outer_radius )   as float
  DIM h_2 as float
  DIM vol_large_tri as float
  DIm  vol_cylinder as float
  DIM  vel_small_tri as float
 
  
  h_2 = (advance_depth_sample*outer_radius)/(abs(outer_radius-inner_radius))
  rem volume  for large triangular cylinder
  vol_large_tri = (1/3)*outer_radius*outer_radius*h_2
  
  rem volume for small  triangular cylinder
  vel_small_tri = (1/3)*outer_radius*inner_radius*(h_2-advance_depth_sample)
  
  rem volume of cylinder
  vol_cylinder = pi*inner_radius*inner_radius*advance_depth_sample
  Volume_triangular = vol_large_tri - vel_small_tri - vol_cylinder
  
  
endsub

rem subroutine to calculate advance depth based on position of cone

rem input args
rem         LVDT_response_model_slope: slope for LVDT response model
rem         LVDT_response_model_intercept: intercept for lVDT response model
rem         Volume_triangular: 
rem         pi: pi const
rem         WegInd_Digits:
rem         advance_depth_sample:
rem         inner_radius:
rem         outer_radius:
sub triangular_additional_volume(LVDT_response_model_slope,LVDT_response_model_intercept, Volume_triangular,pi,WegInd_Digits, advance_depth_sample,inner_radius,outer_radius ) as float
  rem define current position of cone in mm
  DIM cone_position_mm as float
  
  cone_position_mm = (-420/33076) * WegInd_Digits+623 
  
  ' calculate advance depth of sample
  advance_depth_sample = LVDT_response_model_slope*cone_position_mm + LVDT_response_model_intercept

  
  if (advance_depth_sample >= 0) then
    
    rem  calculate additional volume
    addition_triangular_vol(Volume_triangular,advance_depth_sample,inner_radius,outer_radius )
    
    
  endif 




endsub


rem outer radius calculation based on measured side volume
rem input args:
rem     initial_sidepress_digits: initial side pressure, which is pressure we start VCPT or CPT testing
rem     SidePress_Digits: side pressure in digits
rem     r2: outer radius
rem     h_p: cone position plus advanced length derived from linear model
rem     WegInd_Digits: distance in digits
rem     SpindelB_Digits: side volume in digits
rem     r1: inner radius, fixed value
rem     pi: pi value, 3.1415926
rem     LVDT_response_model_slope: slope parameter for LVDT model
rem     LVDT_response_model_intercept: intercept parameter for LVDT model

sub r2_calculation(initial_sidepress_digits,SidePress_Digits,r2,h_p,WegInd_Digits,SpindelB_Digits,r1,pi,LVDT_response_model_slope,LVDT_response_model_intercept) as float
  
  rem define current position of cone in mm
  DIM cone_position_mm as float
  rem subterm definitions
  DIM subterm_1 as float
  DIM subterm_2 as float
  DIM subterm_3 as float
  rem define side volume correction term
  DIM side_volume_correction as float
  rem define side_pressure in Kpa
  DIM side_press_KPa as float
  rem define side volume in ml
  DIM side_solume_ml as float
  rem define side prssure gradient
  DIM side_press_gradient as float 
  rem calculate side pressure in kpa
  side_press_KPa = ((12000 / 65536) * (SidePress_Digits)) - 6000
  rem calculate correction term for expansion of chamber, unit: kpa
  side_press_gradient =  (12000 / 65536) *(SidePress_Digits - initial_sidepress_digits)
  rem the correction is only used with 1st order term
  side_volume_correction = 0.17* side_press_gradient 
  '  -2.43557930e-5*side_press_KPa*side_press_KPa + 2.08693910e-1*side_press_KPa 
  '   -5.95018504e1
  rem change in side volume
  side_solume_ml = (0.03107* SpindelB_Digits) + side_volume_correction
  
  
  cone_position_mm = (-420/33076) * WegInd_Digits+623 
  
  ' calculate advance depth of sample
  h_p = cone_position_mm + LVDT_response_model_slope*cone_position_mm + LVDT_response_model_intercept
  
  rem calculate subterms
  
  subterm_1 = 9*(cone_position_mm + 2*h_p)*(cone_position_mm + 2*h_p)*pi*r1*r1
  
  subterm_2 = 12*(cone_position_mm + 3*h_p)*1000*side_solume_ml
  subterm_3 = 2*( cone_position_mm + 3*h_p)*sqrt(pi)
  
  
  rem calculate outer radius based on adavanced depth h_p and side volume change with correction
  r2 = ((-cone_position_mm)*sqrt(pi)*r1 + sqrt(subterm_1 + subterm_2))/(subterm_3)
  
  
    
  
endsub












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
Sub New_BC5_model_additionside_pressure(initial_sidepress_digits,SidePress_Digits,r2,h_p,r1,SpindelB_Digits,Volume_triangular,LVDT_response_model_slope,LVDT_response_model_intercept, initial_cone_position,initial_strain,start_penetrat_depth,pi,r_0,Additionalside_pressure_array[],local_counter,Side_Addtion_Digits,Side_Addtion_Phys,actual_sidevolume,strain_linear_slope, delta_r, strain_linear_intercept,Actual_volume,initial_volume,linear_calibration_flag,lowestWeg,WegInd_Digits,localtimestep,weginitial_depth,initial_piston_position)      as float
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
    rem record initial side pressure
    initial_sidepress_digits = SidePress_Digits
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
      
      rem the actual volume is initial volume plus deformaton, during penetration the side volume is
      rem decreasing, 
      Actual_volume =  absf(1000*((0.03107* (actual_sidevolume )))) 
            
      '      triangular_additional_volume(LVDT_response_model_slope,LVDT_response_model_intercept,Volume_triangular,pi,WegInd_Digits, advance_depth_sample,inner_radius,outer_radius )
      rem calculate initial volume in physical values
      initial_volume = absf(pi*r_0*r_0*((-420/33076) *(WegInd_Digits-weginitial_depth))) 
      
      rem calculate outer radius r2
      r2_calculation(initial_sidepress_digits,SidePress_Digits,r2,h_p,WegInd_Digits,actual_sidevolume,r1,pi,LVDT_response_model_slope,LVDT_response_model_intercept) 
    
      rem calculate change of strain
      if (initial_volume >0) then
        rem calculate strain ratio r/r0
        '        delta_r = Actual_volume/initial_volume
        
        delta_r = abs((r2-r1)/r1)
      ENDIF
        
      
        
                   
      rem calculate desired side pressure
      Side_Addtion_Phys = strain_linear_slope*(delta_r) + strain_linear_intercept
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
      
      rem calculate outer radius r2
      r2_calculation(initial_sidepress_digits,SidePress_Digits,r2,h_p,WegInd_Digits,actual_sidevolume,r1,pi,LVDT_response_model_slope,LVDT_response_model_intercept) 
  
      rem additional volume based on linear model from LVDT response
            
            

            
            
      rem calculate change of strain
      if (initial_volume >0) then
        rem calculate strain ratio r/r0
        '        delta_r = Actual_volume/initial_volume
        delta_r =  abs((r2-r1)/r1)
               
        
      ENDIF
      rem apply additional side pressure
      rem calculate desired side pressure
      Side_Addtion_Phys = strain_linear_slope*(delta_r) + strain_linear_intercept
      rem calculate desired side pressure in digital values
      Side_Addtion_Digits = Side_Addtion_Phys * 5.461333333    
          
         
            
    endif
    
     
  endif

  

endsub
