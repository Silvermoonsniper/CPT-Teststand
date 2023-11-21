'<ADbasic Header, Headerversion 001.001>
'<Header End>





rem kalman filter implementation for pressure control

sub Kalman_filter (K_1[],K_2[],measured_press_rate,measured_press,dt,r_t[],pressure_rate_prior,pressure_rate_estimate,press_digits_prior,press_digits_estimate,predict_error_cov_prior, predict_error_cov,  predict_error_cov_vel_prior,predict_error_cov_vel)   as float
  
  rem a prior pressure estimate
 
  press_digits_prior = press_digits_estimate + dt*pressure_rate_estimate
 
  pressure_rate_prior = pressure_rate_estimate 
  rem a prior predict error covariance estimate
  predict_error_cov_prior =  predict_error_cov  + dt*dt*predict_error_cov_vel + r_t[2]
  predict_error_cov_vel_prior = predict_error_cov_vel
  
  rem calculate kalman gain
  K_1[1] = predict_error_cov_prior/(predict_error_cov_prior+r_t[1])
  '  -r_t[2]*predict_error_cov_vel_prior*dt*dt
  K_1[2] = 0 
  '  -r_t[1]*dt*predict_error_cov_vel_prior

  
  K_2[1] = predict_error_cov_vel_prior/(predict_error_cov_vel_prior+r_t[2])
  '  -r_t[2]*predict_error_cov_vel_prior*dt
  K_2[2] = 0
  '  predict_error_cov_prior-r_t[1]
  rem update estimate pressure
  
  press_digits_estimate = press_digits_prior + K_1[1]*(measured_press - press_digits_prior) 
  
  predict_error_cov = (1 - K_1[1] )*predict_error_cov_prior 
   
endsub
