
rem calculate fitted velocity
Sub Fit_vel_calcul(linear_coff[],controlstart_step,fittedlinear_coff[],fitted_vel,WegInd_RegOut ) as float
  if (controlstart_step >= 0) then
    fittedlinear_coff[1] = linear_coff[1]
    fittedlinear_coff[2] = linear_coff[2]
         
  endif
    
  rem obtain fitted velocity from linear regression
      
     
  fitted_vel = fittedlinear_coff[1]*WegInd_RegOut+fittedlinear_coff[2] 


endsub 
