function ukoop_final = IRLS(N,m,CN_koop,beta_koop)
w(1:N*m) = 1;  % initial weights assigned at j = 0th iteration 
eps = 1;       % initial threshold assigned at j = 0th iteration
eps_tild = 5;  % Threshold for checking Success or Failure
jmax = 5;      % No of iterations required

for j = 0:jmax
    for k = 0:N-1
        W(:,:,k+1) = diag(w(1,k*m+1:k*m+m));
    end
    W_tild = blkdiag(W(:,:,1:N));
    [o,p] = size(CN_koop'*(W_tild)^(-1));
    ukoop = ((W_tild)^(-1))*((CN_koop'*(W_tild)^(-1))')*((CN_koop'*(W_tild)^(-1) + eye(p))^(-1))*((CN_koop'*(W_tild)^(-1))')*beta_koop;
    eps = min(eps,norm(ukoop,"inf"));
    
    for l = 1:N*m
        w(l) = (ukoop(l)^2 + eps^2)^(-1/4);
    end

    if eps>=0 && eps<=eps_tild
        disp("Success")
    end

end

if eps<=0 || eps>=eps_tild
    disp("Failure")
end
ukoop_final = ukoop;

  
