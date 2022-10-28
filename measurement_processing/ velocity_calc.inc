rem distance data smoothing with moving average (now commented)
rem input args
rem  distance_data[]: array to store distance data
rem  realdistance_data[]: array to store smoothed distance data
rem  intervalp: interval (number of points) for velocity calculation
rem  filter_array[]: smoothed two-element distance data array
rem  Programm_counter: counter
rem  pointer_average: pointer indicates current index of velocity data
rem  windowsize: window size for average filteirng
rem  WegInd_Digits: measured distance data

Sub Distance_data_smooth(distance_data[], realdistance_data[],intervalp,filter_array[],Programm_counter,pointer_average,windowsize,WegInd_Digits) as float
  
  rem modified by zhengyu 2022-06-08########
      
    
  rem record distance data
  rem for the first 'data_sizetrain' data
  if (pointer_average <= windowsize-1) then
    rem transform digital position reading into mm
    inc(pointer_average) 
    distance_data[pointer_average] =  WegInd_Digits 
        
       
    rem smooth distance sensor measurement by moving average
    '    if (Programm_counter > windowsize) then 
    '              
    '      if(pointer_average < windowsize) then
    '        filter_array[2] = filter_array[1] + (distance_data[pointer_average]-distance_data[pointer_average+1])/windowsize
    '      else
    '        filter_array[2] = filter_array[1] + (distance_data[windowsize]-distance_data[1])/windowsize
    '      endif 
    '      filter_array[1] = filter_array[2]
    '    else
    '      filter_array[2] =  WegInd_Digits 
    '      filter_array[1] = filter_array[2]
    '    endif
     
    rem when number of velocity data exceeds size of velocity data array, we use ring buffer to insert new velocity data and remove old one
  else
    rem spilt distance data buffer into small piece, for each piece calculat its average portion
       
    pointer_average = 0
      
        
  endif
  
  rem record distance data for velocity calculation
      
  '       
  '  if (Programm_counter <= intervalp) then
  '    realdistance_data[Programm_counter] = filter_array[2] 
  '    rem when number of velocity data exceeds size of velocity data array, we use ring buffer to insert new velocity data and remove old one
  '  else
  '    ringbuffer(filter_array[2],realdistance_data,intervalp)
  '             
  '  endif
  
 
  
endsub




rem subroutine for distance data smoothing with linear regression
rem time_array[]: x array for regression
rem vel_local: local velocity
rem intervalp: intervap for velocity calculation
rem data_sizetrain: size of velocity data array
rem realdistance_data[]: array to store smoothed distance data
rem Programm_counter: counter for programm cycles
rem dis_regression_full: flag to indicate if data array for regression is full
rem pointer_average: index for current velocity in velocity data array
rem reg_Size: size for regression
rem distance_regression_array[]: distance data array used for regression 
rem reg_para[]: regression parameters
rem newdistance_regression_array[]: distance array used in regression calculation with incremental approach
rem newtime_array[]: x array used in regression calculation with incremental approach
rem distance_subxmean: x mean
rem distance_subymean: distance measurement mean
rem distance_subxydiffpsum: sum product of x and distance measurement
rem distance_subxsquaresum: sum square product of x
rem distpointer_vel_regression: pointer for regression
rem Abtastfrequenz: sampling frequency of adwin
rem Sinusfrequenz_Vibro: vibration frequency
rem WegInd_Digits: distance data measurement
rem pointer_average: local pointer 
sub distance_data_regression (time_array[],vel_local,intervalp,data_sizetrain,realdistance_data[],Programm_counter,dis_regression_full,pointer_average, reg_Size,distance_regression_array[],reg_para[],newdistance_regression_array[],newtime_array[],distance_subxmean,distance_subymean,distance_subxydiffpsum,distance_subxsquaresum,distpointer_vel_regression,Abtastfrequenz,Sinusfrequenz_Vibro,WegInd_Digits,pointer_average)  as float
  
  
  
  '  
  '  rem record distance data into array for regression
  '                           
  if  (pointer_average <= reg_Size-1 ) then 
     
    inc(pointer_average) 
    distance_regression_array[pointer_average] =  WegInd_Digits
   
    time_array[pointer_average] = pointer_average 
                              
  else
    rem reinitialize index
    pointer_average = 0
  endif 
  
  rem linear regression with incremental update
  sub_blockcalculation(estimated_slope_array,estimated_intercept_array,estimated_parameter_pointer,Abtastfrequenz,Sinusfrequenz_Vibro,dis_regression_full,pointer_average, reg_Size,distance_regression_array,reg_para,newdistance_regression_array,newtime_array,distance_subxmean,distance_subymean,distance_subxydiffpsum,distance_subxsquaresum,distpointer_vel_regression,pointer_average, WegInd_Digits,0,reg_Size) 
  
  
  rem replace slope as smoothed distance
  
  '  vel_local = reg_para[2]
  rem record distance data for velocity calculation
      
  '       
  if (Programm_counter <= intervalp) then
    inc(Programm_counter) 
    realdistance_data[Programm_counter] = reg_para[2]
   
   
    rem when number of velocity data exceeds size of velocity data array, we use ring buffer to insert new velocity data and remove old one
  else
    '    ringbuffer(reg_para[2],realdistance_data,intervalp)
    Programm_counter = 0
     
  endif
  if (Programm_counter > 0) then
    vel_local = velocity_dataval(realdistance_data,Programm_counter,intervalp)
  endif
 

  

endsub
