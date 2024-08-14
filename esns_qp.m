function [sData,ddq] = esns_qp(A,bprime,C,dmax,dmin,ur,sm)

nTask=size(A,1);
nJnt=size(A{1},2);
alpha = 1e-5;
A_prev=zeros(1,nJnt);
bprime_prev=0;

Ci=[];
dmaxi=[];
dmini=[];
Ohm=blkdiag(alpha * eye(nJnt), 1.0);
for iTask=1:nTask

    Lambdai=[[A{iTask},bprime{iTask}];[A_prev,zeros(size(A_prev,1),1)]];
    Betai=[(1+sm)*bprime{iTask}-A{iTask}*ur{iTask};bprime_prev-A_prev*ur{iTask}];


    Ci=[Ci;C{iTask}];
    dmaxi=[dmaxi;dmax{iTask}];
    dmini=[dmini;dmin{iTask}];

    Gammai=[[-Ci,zeros(size(Ci,1),1)];[Ci,zeros(size(Ci,1),1)];[zeros(1,size(Ci,2)),1];[zeros(1,size(Ci,2)),-1]];
    Deltai=[-dmini+Ci*ur{iTask};dmaxi-Ci*ur{iTask};1;0];

    options = optimoptions('quadprog','display','None');
    [zSoln, ~, exitCode]=quadprog(Ohm,[],Gammai,Deltai,Lambdai,Betai,[],[],[],options);

    if (exitCode == 1)
        si = 1+sm-zSoln(end);
        si=si-sm;
        Lambdai=[[A{iTask},si*bprime{iTask}];[A_prev,zeros(size(A_prev,1),1)]];
        Betai=[si*bprime{iTask}-A{iTask}*ur{iTask};bprime_prev-A_prev*ur{iTask}];
        [zSoln, ~, ~]=quadprog(Ohm,[],Gammai,Deltai,Lambdai,Betai,[],[],[],options);

        dqi = zSoln(1:nJnt)+ur{iTask};

        if iTask==1
            A_prev=A{iTask};
            bprime_prev=A{iTask}*dqi;
        else
            A_prev=[A_prev;A{iTask}];
            bprime_prev=[bprime_prev;A{iTask}*dqi];
        end
    elseif (exitCode < 1 && iTask > 1)


        si = 0;
        dqi =  dqData{iTask-1};

    else

        dqi = zeros(nJnt,1);
        si = 0;
        A_prev=zeros(size(A{iTask}));
        bprime_prev=zeros(size(bprime{iTask}));

    end


    sData(iTask) = si;
    dqData{iTask} = dqi;


end
ddq = dqi;