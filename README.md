# Robot Arm Controller

This repo contains a stereo camera inverse kinematic control algorithm. 

$$\begin{algorithm}
\
\caption{calculates $\theta$ with linear regression smoothing }
\begin{algorithmic} [1]
\State prevAngles = deque([0,\ldots,0],maxlen=N) 
\State maxDiff = D \Comment{max deviation allowed from linreg}

\Function {linreg$\theta$}{N,prevAngles} \Comment{$\theta$ is generic  for $x,z$}
    \State t = [1,2,\ldots,N]
    \State $\theta$ = prevAngles
    \State model = LinearRegression(t, $\theta$)
    
    \State $\theta$pred = model.predict(N+1)
    \State $\theta$slope = model.slope
\EndFunction

\Function {closest$\theta$Root}{[A,B,C]}
    % \State $z= \pm\arcsin\left(\frac{A}{\sin{x}}\right),\pm\arccos\left(\frac{-B}{\sin{x}}\right)$
    \State Roots = [($\theta_1$, abs($\theta$pred-$\theta_1$)),\ldots,($\theta_k$, abs($\theta$pred-$\theta_k$))]
    
    \Return $\theta_i$ such that abs($\theta$pred-$\theta_i$)) is minimum
\EndFunction

\While {running detection}
    \State{$x = \pm\arccos{C}$}
    \If{time $0\leq T$  } \Comment{initialize until time $T$}
        \State{$z = \arccos{(-B/\sin{x}})$}
        \State {prevAngles.append(z)}
    \EndIf
\algstore{myalg}

\end{algorithmic}
\end{algorithm}$$
