function uhatd = uhat_d(u,ud)
uhat = u/norm(u);
uhatd = (ud - dot(uhat,ud)*uhat)/norm(u);