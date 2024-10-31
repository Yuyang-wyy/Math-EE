function K = lqr_k(L0)
%LQR_K
%    K = LQR_K(L0)

%    This function was generated by the Symbolic Math Toolbox version 24.1.
%    2024-08-20 17:43:54

t2 = L0.^2;
t3 = L0.^3;
mt1 = [L0.*(-5.192040150357123)-t2.*3.947557984589978e+1+t3.*7.750787227635803e+1+2.874081614917563e-2];
mt2 = [L0.*(-2.00924403304329e-2)+t2.*1.430477640888537e-1-t3.*3.663647595883324e-1+3.039779762975084e-3];
mt3 = [L0.*(-7.146304319955089e-1)-t2.*7.748598834029472+t3.*9.905882900477971+6.396647515107512e-3];
mt4 = [L0.*2.577641037770936e-3-t2.*1.166374156636816e-2+t3.*2.738151767787285e-2+4.82271002829299e-5];
mt5 = [L0.*(-2.655753474315045)-t2.*1.297693582128775e+1+t3.*3.637426768448702e+1+3.273876710153847e-2];
mt6 = [L0.*(-1.635622527772511e-1)+t2.*1.145641811247298-t3.*3.028859105159021+1.206083147749403e-2];
mt7 = [L0.*(-2.428860544123773)-t2.*1.012799560778419e+1+t3.*2.569519199500538e+1+2.04665102733784e-2];
mt8 = [L0.*(-1.029097099234966e-1)+t2.*7.506647349344368e-1-t3.*2.029221450514235+7.19226499852321e-3];
mt9 = [L0.*(-2.094058676145232e-1)+t2.*1.359949728975021-t3.*3.437676921837593+1.779227163452606e-2];
mt10 = [L0.*8.169891366451616e-4-t2.*6.429510037702442e-3+t3.*1.79475181991222e-2+2.13433292919481e-2];
mt11 = [L0.*(-6.601215872771936e-1)+t2.*4.291248574054439-t3.*1.085165682880497e+1+5.637827513006114e-2];
mt12 = [L0.*2.6827347599255e-3-t2.*2.115345808365963e-2+t3.*5.912608782640789e-2+6.759578217886525e-2];
K = reshape([mt1,mt2,mt3,mt4,mt5,mt6,mt7,mt8,mt9,mt10,mt11,mt12],2,6);
end