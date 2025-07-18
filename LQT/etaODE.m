function deta = etaODE(t, eta, A, B, C, Q, R, P_interp, r_func)
    P_t = reshape(P_interp(t), size(A));
    Acl = A - B*(R^-1)*B'*P_t;
    r_t = r_func(t);
    deta = -Acl'*eta - C'*Q*r_t;
end
