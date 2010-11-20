%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Iterative algorithm YALMIP/SeDuMi version

clc; clear all;
opts = sdpsettings('solver','sedumi','verbose',0,'warning',1);
opts.sedumi.eps = 1e-7;
disp(' ');
stop_crit = 1e-4;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONTROL SYSTEM MODEL  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

n = 3;          % problem size
nu = 1;         % number of switching modes
ni = 1;         % number of input signals
no = 2;         % number of output signals
nn=4;           % number of noises
t = 0.015;       % discretization period
alpha=0.0556;
margin=sqrt(1+alpha);



% nominal matrices A
AC{1} = [0,1,0 ; -4.2,-1.5,4.2	; 0.77,0,-0.77	];
AC{2} = [0,1,0 ; -7.1,-1.9,7.1	; 1,0,-1		];
AC{3} = [0,1,0 ; -78,-4.1,78		; 2.8,0,-2.8	];
AC{4} = [0,1,0 ; -4,-1.4,4		; 0.62,0,-0.62	];
AC{5} = [0,1,0 ; -116,-2.36,116	; 2.3,0,-2.3	];
AC{6} = [0,1,0 ; -7.9,-1.1,7.9	; 0.56,0,-0.56	];
AC{7} = [0,1,0 ; -55,-0.66,55	; 0.84,0,-0.84	];
AC{8} = [0,1,0 ; -14.5,-0.43,14.5; 0.33,0,-0.33	];
AC{9} = [0,1,0 ; -18,-0.31,18	; 0.34,0,-0.34	];

% nominal matrices B
BC{1} = [0;-7.4;0];
BC{2} = [0;-12.7;0];
BC{3} = [0;-57;0];
BC{4} = [0;-7.5;0];
BC{5} = [0;-42;0];
BC{6} = [0;-13.8;0];
BC{7} = [0;-22.5;0];
BC{8} = [0;-8.6;0];
BC{9} = [0;-10;0];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% WEIGHT MATRICES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Qi and Ri
%POLY = [-9 -20 -25];
POLY = [-45 -10+5i -10-5i];
%POLY = [-5 -5+5i -5-5i];
for i = 1:nu,
    AC{i}=AC{3};
    BC{i}=BC{3};
    Q{i} = weight(AC{i},BC{i},POLY);
    R{i} = 1;
end;

% recompute A, B, C for discrete case
for i = 1:nu,
    
    [A{i} B{i} Q{i} R{i}] = cont2disc(AC{i},BC{i},Q{i},R{i},t);

end;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% NOISE MATRICES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:nu,
    % noise intensities
gamma{i,1} = 2.65;
gamma{i,2} = 2.65;
gamma{i,3} = 2.65;
gamma{i,4 }= 2.65;
    AN{i,1}(1,1) = 0;
    AN{i,1}(1,2) = 0;
    AN{i,1}(1,3) = 0;
    AN{i,1}(2,1) = AC{i}(2,1)*t;
    AN{i,1}(2,2) = 0;
    AN{i,1}(2,3) = AC{i}(2,3)*t;
    AN{i,1}(3,1) = 0;
    AN{i,1}(3,2) = 0;
    AN{i,1}(3,3) = 0;
    BN{i,1}(1,1) = 0;
    BN{i,1}(2,1) = 0;
    BN{i,1}(3,1) = 0;
    
    AN{i,2}(1,1) = 0;
    AN{i,2}(1,2) = 0;
    AN{i,2}(1,3) = 0;
    AN{i,2}(2,1) = 0;
    AN{i,2}(2,2) = AC{i}(2,2)*t;
    AN{i,2}(2,3) = 0;
    AN{i,2}(3,1) = 0;
    AN{i,2}(3,2) = 0;
    AN{i,2}(3,3) = 0;
    BN{i,2}(1,1) = 0;
    BN{i,2}(2,1) = 0;
    BN{i,2}(3,1) = 0;
    
    AN{i,3}(1,1) = 0;
    AN{i,3}(1,2) = 0;
    AN{i,3}(1,3) = 0;
    AN{i,3}(2,1) = 0;
    AN{i,3}(2,2) = 0;
    AN{i,3}(2,3) = 0;
    AN{i,3}(3,1) = AC{i}(3,1)*t;
    AN{i,3}(3,2) = 0;
    AN{i,3}(3,3) = AC{i}(3,3)*t;
    BN{i,3}(1,1) = 0;
    BN{i,3}(2,1) = 0;
    BN{i,3}(3,1) = 0;
    
    AN{i,4}(1,1) = 0;
    AN{i,4}(1,2) = 0;
    AN{i,4}(1,3) = 0;
    AN{i,4}(2,1) = 0;
    AN{i,4}(2,2) = 0;
    AN{i,4}(2,3) = 0;
    AN{i,4}(3,1) = 0;
    AN{i,4}(3,2) = 0;
    AN{i,4}(3,3) = 0;
    BN{i,4}(1,1) = 0;
    BN{i,4}(2,1) = BC{i}(2,1)*t;
    BN{i,4}(3,1) = 0;
end
for i = 1:nu,
A{i}=A{i}*margin;
B{i}=B{i}*margin;
end;
%-----------------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%% STATE FEEDBACK CONTROL SYNTHESIS %%%%%%%%%%%%%%%%%%%%%%%% 
%-----------------------------------------------------------------------------------
%initialization
C = [1,0,0; 0,1,0];
X0=eye(n);
[U,S,V] = svd(C);
  V2 = V(:,no+1:n);

quiz = set([]);
 
	X = sdpvar(n,n);
    Y= sdpvar(ni,n);     

    quiz = quiz + set(X > 0);
    for i = 1:nu,
    quiz = quiz + set([-X, X, (A{i}*X-B{i}*Y)', gamma{i,1}*(AN{i,1}*X-BN{i,1}*Y)', gamma{i,2}*(AN{i,2}*X-BN{i,2}*Y)', gamma{i,3}*(AN{i,3}*X-BN{i,3}*Y)', gamma{i,4}*(AN{i,4}*X-BN{i,4}*Y)';
                        X, -inv(Q{i}), zeros(n,n), zeros(n,n), zeros(n,n), zeros(n,n), zeros(n,n); 
                        (A{i}*X-B{i}*Y), zeros(n,n),-X, zeros(n,n), zeros(n,n), zeros(n,n), zeros(n,n); 
                        gamma{i,1}*(AN{i,1}*X-BN{i,1}*Y), zeros(n,n), zeros(n,n),-X, zeros(n,n), zeros(n,n), zeros(n,n);
                        gamma{i,2}*(AN{i,2}*X-BN{i,2}*Y), zeros(n,n), zeros(n,n), zeros(n,n), -X, zeros(n,n), zeros(n,n);
                        gamma{i,3}*(AN{i,3}*X-BN{i,3}*Y),zeros(n,n), zeros(n,n), zeros(n,n), zeros(n,n), -X, zeros(n,n);
                        gamma{i,4}*(AN{i,4}*X-BN{i,4}*Y),zeros(n,n), zeros(n,n),zeros(n,n), zeros(n,n), zeros(n,n), -X]<0);
    
end;  


minimize = [];

quiz = solvesdp(quiz,minimize,opts);
if (quiz.problem), disp('Infeasible to find X'); return; end;

disp('Found feasible solutions X > 0');
X=double(X);
Y=double(Y);
P=inv(X)
K0=Y*P
%end;
%-----------------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%% OUTPUT FEEDBACK CONTROL SYNTHESIS %%%%%%%%%%%%%%%%%%%%%%%% 
%-----------------------------------------------------------------------------------      
% matrix C
C = [1,0,0; 0,1,0];
X0=eye(n);
[U,S,V] = svd(C);
  V2 = V(:,no+1:n);

result = K0*V2;
K=K0;
iteration = 0;
 
while norm(result,'fro') >= stop_crit
    Yn= sdpvar(n,n);
 Pn= sdpvar(n,n);
for i=1:nu
    for j=1,nn;
sp{i,j} = gamma{i,j}^2*(AN{i,j}-BN{i,j}*K)'*Pn*(AN{i,j}-BN{i,j}*K);
sy{i,j} = gamma{i,j}^2*(AN{i,j}-BN{i,j}*K)*Yn*(AN{i,j}-BN{i,j}*K)';
end;
end;
for i=1:nu
    sump{i}=0;
    sumy{i}=0;
    for j=1,nn;
    sump{i} = sump{i}+sp{i,j};
    sumy{i}= sumy{i} + sy{i,j};
end;
end;
F = set([]);
YE=zeros(n,n);
for i=1:nu
 YE=YE+(A{i}-B{i}*K )*Yn *(A{i}-B{i}*K )' + sumy{i};
end
F=set(YE+X0-nu*Yn ==0)+set(Yn>0);
solvesdp(F,minimize,opts);
Yn=double(Yn);
G= set([]);
G = G + set(Pn > 0);
    for i = 1:nu,
    G = G + set((A{i}-B{i}*K )'*Pn *(A{i}-B{i}*K ) + sump{i}+Q{i}+K'*R{i}*K-Pn <0);
end;
solvesdp(G,minimize,opts);
Pn=double(Pn);

for i=1:nu
    for j=1,nn;
sbpb{i,j} = gamma{i,j}^2*BN{i,j}'*Pn*BN{i,j};
sbpa{i,j} = gamma{i,j}^2*BN{i,j}'*Pn*AN{i,j};
end;
end;
for i=1:nu
    sumbpb{i}=0;
    sumbpa{i}=0;
    for j=1,nn;
    sumbpb{i} = sumbpb{i}+sbpb{i,j};
    sumbpa{i}= sumbpa{i} + sbpa{i,j};
end;
end;
sumk1=zeros(ni,ni);
sumk=zeros(ni,n);
for i=1:nu
 sumk1=sumk1+R{i}+B{i}'*Pn*B{i}+sumbpb{i};
 sumk=sumk+B{i}'*Pn*A{i}+sumbpa{i};
end
Kn=inv(sumk1)*sumk*(eye(n)-V2*inv(V2'*inv(Yn)*V2)*V2'*inv(Yn));
deltaK=inv(sumk1)*sumk*(eye(n)-V2*inv(V2'*inv(Yn)*V2)*V2'*inv(Yn))-K;
beta =sdpvar(ni);
qr= set([]);
qr =qr + set(beta > 0)+set(beta-1.9<0);
    for i = 1:nu,
    qr = qr + set([-Pn+sump{i}+Q{i} ,(A{i}-B{i}*(K+beta*deltaK) )'*Pn; 
    Pn*(A{i}-B{i}*(K+beta*deltaK)),-Pn ]<0);
end;
solvesdp(qr,-beta,opts);
beta=double(beta)

%beta=0.3;
K=K+beta*deltaK;
result = K*V2;
normfrokv2=norm(result,'fro')


iteration = iteration + 1;
%if iteration >1 end
ploty(iteration) = norm(result,'fro');
beta_vec(iteration) = beta;
end

norm(result,'fro')

plot(ploty,'-')
title('Norm');
plot(beta_vec,'-')
title('Beta');
figure
plot(ploty,'-')
title('Norm');
figure
K
FC = K*pinv(C)
evy=eig(double(Yn))
evP=eig(double(Pn))
for i = 1:nu,
%evAC=eig((A{i}-B{i}*FC*C)*sqrt(1.004))
evAC=eig((A{i}-B{i}*FC*C)*(1/margin))
end
for i = 1:nu,
A{i}=A{i}*(1/margin);
B{i}=B{i}*(1/margin);
delta=sqrt((alpha*gamma{i,1}^2)/(nn*(alpha+1)))
end;
for i = 1:nu,
   	    dstep(A{i}-B{i}*FC*C, B{i}*FC*C,[1 0 0],0,1);
  	    hold on;
   	end;
   	grid on;
   	grid minor;
end;