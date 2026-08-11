# Certifiably Optimal Solvers and Theoretical Properties of SLAM

David M. Rosen, Kasra Khosoussi, Connor Holmes, Gamini Dissanayake, Timothy Barfoot, and Luca Carlone

Chapters 1–3 have discussed how to formalize the estimation problems arising in SLAM as optimization problems, using maximum a posteriori estimation or (more generally) M-estimation. Moreover, these chapters have introduced iterative local solvers (e.g., Gauss-Newton, Levenberg-Marquardt, gradient descent) that look for a solution of the resulting optimization by iteratively refining a given initial guess.

In this chapter, we take a closer look at the optimization problems arising in SLAM and address two fundamental questions. First, can we design eficient algorithms that are guaranteed to compute a globally optimal solution of these problems, possibly without an initial guess? And second, how accurate is the optimal solution as an estimate of the ground truth, and what factors afect its accuracy?

The first question is about computation and reliability: the question starts from the observation that SLAM requires solving nonconvex optimization problems, which have multiple local minima. Depending upon the quality of the initial guess, iterative algorithms can get stuck in local minima, and hence produce incorrect estimates (Figure 6.1). Moreover, iterative methods do not provide tools to detect convergence to suboptimal solutions, which leads to trustworthiness and reliability concerns in practical applications. In Section 6.1, we show that despite the nonconvexity of typical SLAM problems, we can design certifiably optimal algorithms that solve the SLAM optimization to provable optimality, and can discern sub-optimal solutions from optimal ones. These algorithms are based on a mathematical tool, known as semidefinite relaxation, which we review in Section 6.1.1. The chapter presents certifiable algorithms for PGO (SE-Sync, Section 6.1.2), landmark-based SLAM with range and bearing measurements (Section 6.1.3), and then discusses extensions to other SLAM problems, including problems with range-only measurements, anisotropic noise, and outliers (Section 6.1.4).

The second question is concerned with fundamental limits and estimation errors: when designing a SLAM system, one often wants to ensure that the robot pose estimate and the map estimate are close to the ground truth, since large errors may induce failure in downstream tasks, including motion planning. Understanding fundamental limits is not only useful for analytical purposes but has very practical implications: as we discuss below, these limits are influenced by the structure of the factor graph, and we can actively control this structure by carefully driving the robot (e.g., forcing it to revisit places or landmarks). Therefore, in Section 6.2 we discuss information-theoretic limits on the achievable accuracy of the SLAM estimate. In particular, Section 6.2.1 introduces the Cram´er-Rao Lower Bound to quantify the estimation error in SLAM, and Section 6.2.2 draws connections between this bound and the graphical structure of the SLAM problem.

As we will see, investigating these questions reveals deep connections between the algebraic, geometric, and graph-theoretic structures encoded in the SLAM problem, and the computational and statistical hardness of solving it. As usual, we conclude the chapter with an outlook to recent trends and references in Section 6.3.

## 6.1 Certifiably Optimal Solvers for SLAM

SLAM is conventionally formulated as a high-dimensional and nonconvex problem. Finding the global solution to a general nonconvex optimization problem is inherently challenging, primarily due to the existence of numerous local minima (Figure 6.1). In particular, many special cases of SLAM are known to be NP-hard, including, e.g., angular synchronization, rotation averaging, and PGO. It follows that there is no algorithm that is capable of eficiently solving these problems in general, unless P = NP [938].

Despite this theoretical complexity, early SLAM research consistently demonstrated surprising convergence to solutions close to the ground truth. SLAM algorithms are typically initialized with robot poses computed using odometry. In a well-calibrated mobile robot, odometry drift can be reduced to less than one percent of the distance traveled. Thus, the unexpectedly reliable convergence of SLAM was initially attributed to the availability of high-quality initial estimates.

However, subsequent research has shown that even in the presence of poor or inconsistent initializations, a range of optimization techniques, including Stochastic Gradient Descent, Levenberg–Marquardt, and preconditioned conjugate gradient methods, can often recover near-optimal solutions. These findings suggest that, despite being a nonlinear and nonconvex problem, SLAM possesses an intrinsic structure that makes it particularly amenable to solution through specialized optimization strategies. At the same time, these results show that simply applying of-the-shelf local optimization methods (such as gradient descent or quasi-Newton methods) can produce egregiously wrong estimates, even if the underlying instance of the SLAM problem is well-posed. This insight has motivated the development of algorithmic frameworks that explicitly leverage the graph-theoretic and geometric underpinnings of the SLAM problem to obtain improved convergence properties and guarantees.

One of the most exciting recent advances in SLAM has been the development of practical optimization algorithms that —despite this problem’s general intractability are nevertheless provably capable of recovering certifiably globally optimal solutions of common SLAM formulations (e.g., PGO) under mild conditions. These techniques, called certifiably correct optimization methods, are based on convex relaxation (rather than local optimization). Moreover, they are certifiably optimal in the sense that upon computing a solution to the optimization problem, they will be able to quantify how suboptimal that estimate is, and possibly certify its optimality. Such a statement does not contradict the NP-hardness of the problem: in worst-case scenarios, these algorithms can still fail to produce a certificate of optimality for a solution they compute and only provide a suboptimality bound. However, these algorithms remain of practical interest for two main reasons: (i) in practice, they do produce an optimality certificate for virtually all SLAM problems of practical interest (e.g., for reasonable amounts of measurement noise), and (ii) the failure to produce an optimality certificate is in itself informative, since it might trigger a warning to downstream tasks not to trust the SLAM estimate, or for the robot to take fail-safe measures.

![](images/720ec4e3fe91e1e145ab3a6aac31fa34dcd1594e8d859b3846e014998b143526.jpg)  
(a)

![](images/c6dca3d1f2b0dd83049d593099daf7f488224730fc84043b628d17e4cca20887.jpg)  
(b)

![](images/bda6c5b92f87009d964f95a4646905af03831178f9e91fbd7bee4bb6c68c4c37.jpg)  
(c)

![](images/4849de8c3223210a5901000ae243173f9083156dc3a2fb7c986f77aa3396d29a.jpg)  
(d)  
Figure 6.1 Examples of local minima in maximum likelihood estimation for SLAM. This figure shows several local minima for a real-world instance of the pose-graph SLAM problem (6.9) in which a robot navigated through the multi-level parking garage shown in (a). (b) shows the correct (i.e. globally optimal) solution of (6.9) computed using SE-Sync [937], while (c) and (d) show examples of suboptimal local minima obtained from a local optimization method initialized at a randomly-sampled starting point. (Panel (a) reproduced from [400]).

In this section we provide a brief introduction to certifiably correct estimation methods for SLAM. We begin by considering Shor’s relaxation, one of the fundamental tools that is used to construct the convex relaxations underpinning certifiable estimation techniques. Next, we show how to apply Shor’s relaxation to produce a certifiably correct estimation method (SE-Sync) to solve the fundamental problem of PGO. Finally, we discuss discuss extensions of SE-Sync and Shor’s relaxation to a broader range of SLAM problems.

## 6.1.1 Shor’s Relaxation

In this subsection we introduce Shor’s relaxation, one of the fundamental tools that we will use to construct the convex relaxations underpinning certifiable estimators.

In brief, Shor’s relaxation is a procedure for constructing convex relaxations of a Quadratically constrained quadratic program (QCQP); that is, optimization problems in which the objective and constraint functions are quadratics. As we will see in the next section, many common SLAM formulations can be cast as QCQPs.

We introduce Shor’s relaxation by describing its application to a generic QCQP, and then tailor it to SLAM in the next section. Consider the following QCQP:

$$
\begin{array}{r l} p ^ {*} = \min _ {\boldsymbol {x} \in \mathbb {R} ^ {n}} \boldsymbol {x} ^ {\top} \boldsymbol {C} \boldsymbol {x} \\ \mathrm{s.t.} \boldsymbol {x} ^ {\top} \boldsymbol {A} _ {i} \boldsymbol {x} = b _ {i} & i = 1, \ldots , m. \end{array}\tag{QCQP}
$$

where $C , A _ { 1 } , \ldots , A _ { m } \in \mathbb { S } ^ { n }$ are symmetric matrices and $\pmb { b } = ( b _ { 1 } , \dots , b _ { m } ) \in \mathbb { R } ^ { m }$ is a vector. We will show how to produce a convex relaxation of (QCQP) by applying a sequence of simple algebraic manipulations.

To begin, if $M \in \mathbb { S } ^ { n }$ is any symmetric matrix, we may exploit the cyclic property of the trace to rewrite the quadratic form $x ^ { \top } M x$ determined by M according to

$$
\boldsymbol {x} ^ {\top} \boldsymbol {M} \boldsymbol {x} = \operatorname{tr} \left(\boldsymbol {x} ^ {\top} \boldsymbol {M} \boldsymbol {x}\right) = \operatorname{tr} \left(\boldsymbol {M} \boldsymbol {x} \boldsymbol {x} ^ {\top}\right).\tag{6.1}
$$

Applying (6.1) to (QCQP), we thus obtain the equivalent form:

$$
\begin{array}{r l} p ^ {*} = \min _ {\boldsymbol {x} \in \mathbb {R} ^ {n}} \mathrm{tr} \left(\boldsymbol {C x x} ^ {\top}\right) \\ \text {s.t.} \mathrm{tr} \left(\boldsymbol {A} _ {i} \boldsymbol {x x} ^ {\top}\right) = b _ {i}, & i = 1, \ldots , m. \end{array}\tag{6.2}
$$

Now observe that the decision variable x only enters problem (6.2) through outer products of the form $\pmb { X } \triangleq \pmb { x x } ^ { \top } ;$ ; note that every such matrix X is symmetric, rank-1, and positive-semidefinite by construction. Conversely, if $\pmb { X } \in \mathbb { S } _ { + } ^ { n }$ is a positivesemidefinite matrix and rank $( X ) = 1$ , it is easily shown (by considering a symmetric eigendecomposition) that X admits a symmetric factorization of the form $\pmb { X } = \pmb { x } \pmb { x } ^ { \top }$ for some $\pmb { x } \in \mathbb { R } ^ { n }$ . Putting these observations together, we thus have the equivalence

$$
\boldsymbol {X} \in \mathbb {S} _ {+} ^ {n} \text { and } \operatorname{rank} (\boldsymbol {X}) = 1 \quad \Longleftrightarrow \quad \exists \boldsymbol {x} \in \mathbb {R} ^ {n} \text { such   that } \boldsymbol {X} = \boldsymbol {x x} ^ {\top}.\tag{6.3}
$$

In light of (6.3), problem (6.2) is equivalent to

$$
\begin{array}{c} p ^ {*} = \min _ {\boldsymbol {X} \in \mathbb {S} ^ {n}} \operatorname{tr} \left(\boldsymbol {C X}\right) \\ \text {s.t.} \operatorname{tr} \left(\boldsymbol {A} _ {i} \boldsymbol {X}\right) = b _ {i}, \quad i = 1, \ldots , m, \\ \boldsymbol {X} \succeq 0, \\ \operatorname{rank} \left(\boldsymbol {X}\right) = 1. \end{array}\tag{6.4}
$$

Thus far problems (6.2) and (6.4) are completely equivalent; however, formulation (6.4) has the advantage that it reveals a great deal of useful structure. Indeed, the objective and constraint functions in (6.4) are linear functions of the (matrix) decision variable X, and the positive-semidefiniteness constraint $X \succeq 0$ is convex. Thus, the only dificulty in solving (6.4) is due to the (nonconvex) rank constraint.

Shor’s relaxation [1011] simply consists of discarding the rank constraint appearing in (6.4), thereby producing the following convex relaxation of (QCQP):

$$
\begin{array}{c} d ^ {*} = \min _ {\boldsymbol {X} \in \mathbb {S} ^ {n}} \operatorname{tr} (\boldsymbol {C X}) \\ \text {s.t.} \operatorname{tr} (\boldsymbol {A} _ {i} \boldsymbol {X}) = b _ {i}, \quad i = 1, \ldots , m, \\ \boldsymbol {X} \succeq 0. \end{array}\tag{SDP}
$$

Note that (SDP) entails minimizing a linear function over the set of positivesemidefinite matrices, subject to a set of linear equality constraints; problems of this form are called semidefinite programs. Semidefinite programs, or SDPs, are convex optimization problems, and can be solved in polynomial time.

Now let us consider the relation between problem (QCQP) and its convex relaxation (SDP). First, note that we obtained (QCQP) from (SDP) by expanding the former’s feasible set $( i . e . ,$ , by dropping the rank constraint in (6.4)); indeed, it is easy to see that every feasible point $\mathbf { x } \in \mathbb { R } ^ { n }$ in (QCQP) lifts to a corresponding feasible point $X \triangleq x x ^ { \mathsf { T } }$ for (SDP).<sup>1</sup> It follows that the optimal values of (QCQP) and (SDP) satisfy the relation:

$$
d ^ {*} \leq p ^ {*},\tag{6.5}
$$

since the latter problem minimizes the same objective over a larger feasible set.

Inequality (6.5) already provides a very useful method for assessing the quality of candidate solutions of (QCQP). Suppose that we have a feasible point $\hat { \pmb x } \in \mathbb { R } ^ { n }$ of (QCQP); for example, xˆ might have been obtained by performing local optimization. Writing $f ( \pmb { x } ) \triangleq \pmb { x } ^ { \top } C \pmb { x }$ for the objective, inequality (6.5) implies that we may bound the suboptimality $f ( \hat { \pmb x } ) - \boldsymbol p ^ { * }$ of xˆ as a solution of (QCQP) according to

$$
f (\hat {\boldsymbol {x}}) - p ^ {*} \leq f (\hat {\boldsymbol {x}}) - d ^ {*}.\tag{6.6}
$$

Note that while the optimal value of $p ^ { * }$ of (QCQP) is very hard to compute in general, the optimal value $d ^ { * }$ can be computed eficiently by solving the relaxation (SDP). Inequality (6.6) thus gives us a practical way of bounding xˆ’s suboptimality without the need to know $p ^ { * }$ itself. In particular, if the right-hand side of (6.6) is small, we may conclude that x is a near-optimal solution of (QCQP).

Moreover, if we solve the relaxation (SDP) and it so happens that the resulting minimizer $X ^ { * } = x ^ { * } x ^ { * \top }$ has rank 1, then it immediately follows that the vector $\pmb { x } ^ { * } \in \mathbb { R } ^ { n }$ is a global minimizer for the original (nonconvex) problem (QCQP) (since $\pmb { x } ^ { * }$ is feasible in (QCQP) and satisfies $f ( \pmb { x } ^ { * } ) = d ^ { * }$ in (6.6)). As we will see, it turns out that this favorable situation actually occurs quite often for many robotic state estimation tasks, enabling us to recover exact, globally optimal solutions for the nonconvex problem (QCQP) from solutions of its relaxation (SDP).

In practice, solving large SDPs might still be slow or memory intensive with ofthe-shelf solvers, and it is common to develop specialized solvers (e.g., SE-Sync, which we describe below) for large-scale robotics applications.

## 6.1.2 SE-Sync: Certifiably Correct Pose-Graph Optimization

In this subsection we show how one can apply Shor’s relaxation to develop a certifiably correct algorithm (SE-Sync) for solving the fundamental problem of pose-graph optimization (PGO). PGO is one of the simplest and most commonly used SLAM formulations, and thus provides a natural concrete example to illustrate the construction of certifiable estimation algorithms. Moreover, PGO was the first SLAM formulation that was shown to be amenable for convex relaxations [149, 151, 939], and the corresponding ideas have been shown to generalize to a wide range of SLAM problems, as we will discuss below.

Our development proceeds in three stages. First, we show how to formalize PGO as an instance of maximum likelihood estimation, and how to reduce it to a QCQP. Next, we derive Shor’s SDP relaxation for PGO, and (crucially) show that this relaxation is in fact exact for suficiently small measurement noise; this implies that we can recover globally optimal solutions to PGO by solving its (convex) SDP relaxation. Finally, we describe a specialized, structure-exploiting optimization algorithm that enables us to solve large-scale instances of this SDP relaxation in practice.

## 6.1.2.1 Pose-Graph Optimization: QCQP Formulation

Pose-graph optimization (PGO) estimates the values of a set of n unknown poses $T _ { 1 } , \ldots , T _ { n } \in { \mathrm { S E } } ( d )$ in d-dimensional space (typically in SLAM d = 2 or 3), given noisy measurements $\tilde { \mathbf { T } } _ { i j } \approx \mathbf { \cal { T } } _ { i } ^ { - 1 } \mathbf { \cal { T } } _ { j }$ of a set of relative pose measurements between them. In practice, the unknown poses $T _ { 1 } , \ldots , T _ { n }$ describe the trajectory of the robot (i.e., they are sampled at discrete times along the robot trajectory), while the measurements $\tilde { \pmb { T } } _ { i j }$ are obtained by the SLAM front-end, $e . g .$ , through LiDAR scan matching, wheel odometry, or 3D computer vision techniques. In this subsection, we show how to formalize this estimation problem via maximum likelihood estimation. As we will see, under suitable assumptions on the noise, the resulting optimization problem is a QCQP.

![](images/511c5728c6b53730be52eaf0d6bb3feba6c37c575039c7ec017f45cd8f672321.jpg)  
Figure 6.2 An example of pose graph. Here the vertices are in one-to-one correspondence with the unknown poses $T _ { i } = ( t _ { i } , R _ { i } ) \in \mathrm { S E } ( d )$ to be estimated, and the directed edges are in one-to-one correspondence with the set of noisy measurements $\tilde { \pmb { T } } _ { i j }$ ≈ ${ \pmb T } _ { i } ^ { - 1 } { \pmb T } _ { j }$ of the relative poses between them.

To begin, it is often convenient to model the data defining this estimation problem using a pose graph $\overrightarrow { \mathcal { G } } , ^ { 2 }$ constructed as follows. Let $\mathcal { G } = ( \nu , \mathcal { E } )$ be a simple undirected graph whose nodes i are in one-to-one correspondence with the unknown poses $\mathbf { \delta } _ { \mathbf { \mathcal { T } } _ { i } }$ and whose edges $\{ i , j \} \in { \mathcal { E } }$ are in one-to-one correspondence with the set of available measurements.<sup>3</sup> We will assume (without loss of generality) that $\mathcal { G }$ is connected.<sup>4</sup> The pose graph $\vec { \mathcal { G } } = ( \nu , \vec { \mathcal { E } } )$ is then obtained from $\mathcal { G }$ by assigning an orientation for each of the latter’s edges (cf. Figure 6.2). By convention, the measurement $\tilde { \pmb { T } } _ { i j }$ , which describes (a noisy version of) the pose $\mathbf { \delta } _ { T _ { j } }$ in the coordinate frame of the pose $\mathbf { \delta } _ { \mathbf { \mathcal { T } } _ { i } }$ , is associated with a directed edge from i to $j$ .

In order to formalize PGO as a maximum likelihood estimation, we must posit a noise model for the available measurements $\{ \tilde { T } _ { i j } \}$ . To do so, we will make use of the isotropic Langevin distribution $\mathcal { L } ( M , \kappa )$ : this is an exponential family distribution over $\mathrm { S O } ( d )$ whose probability density function is given by

$$
p (\boldsymbol {R}; \boldsymbol {M}, \kappa) = \frac {1}{c _ {d} (\kappa)} \exp \left(\kappa \mathrm{tr} \left(\boldsymbol {M} ^ {\top} \boldsymbol {R}\right)\right),\tag{6.7}
$$

where $M \in \operatorname { S O } ( d )$ and $\kappa \geq 0$ are parameters, and $c _ { d } ( \kappa )$ is a normalization constant. Note that M plays the role of a location parameter (called the mode), while $\kappa \geq 0$ is a scalar concentration parameter. The isotropic Langevin distribution admits a particularly simple generative description in dimensions 2 and 3: to produce a sample $\tilde { \pmb { R } } \sim \mathcal { L } ( \pmb { M } , \kappa )$ , we first sample a rotation angle $\theta \sim$ vonMises $( 0 , 2 \kappa )$ from the von Mises distribution on the circle, and then set $\tilde { \mathbf { R } } = \mathbf { } M \cdot R ( \theta )$ if $d = 2$ , or $\tilde { \pmb { R } } = M \mathrm { e x p } ( \theta \pmb { v } ^ { \wedge } )$ if $d = 3$ , where $\pmb { v } \sim \mathcal { U } ( S ^ { 2 } )$ is a uniformly sampled rotation axis [937]. Intuitively, one can think of this distribution as an analogue of the Gaussian distribution over the (non-Euclidean) manifold of rotations.

Given a pose-graph $\vec { \mathcal { G } } = ( \nu , \vec { \varepsilon } )$ , we will assume that each measurement $\tilde { \pmb { T } } _ { i j } =$ $( \tilde { \pmb { t } } _ { i j } , \tilde { \pmb { R } } _ { i j } ) \in \mathrm { S E } ( d )$ is obtained by sampling from the following probabilistic generative model:

$$
\begin{array}{r l r} & {\tilde {\pmb {t}} _ {i j} = \bar {\pmb {t}} _ {i j} + \pmb {t} _ {i j} ^ {\epsilon}, \qquad \pmb {t} _ {i j} ^ {\epsilon} \sim \mathcal {N} (0, \tau_ {i j} ^ {- 1} I _ {d}), \qquad \forall (i, j) \in \overrightarrow {\mathcal {E}},} \\ & {\tilde {\pmb {R}} _ {i j} = \bar {\pmb {R}} _ {i j} \pmb {R} _ {i j} ^ {\epsilon}, \qquad \pmb {R} _ {i j} ^ {\epsilon} \sim \mathcal {L} (\mathbf {I} _ {d}, \kappa_ {i j}),} \end{array}\tag{6.8}
$$

where $\bar { T } _ { i j } = ( \bar { t } _ { i j } , \bar { R } _ { i j } ) \in \mathrm { S E } ( d )$ is the true (latent) value of the relative pose between $\mathbf { \delta } _ { \mathbf { \mathcal { T } } _ { i } }$ and $\mathbf { \delta } _ { T _ { j } }$ . Model (6.8) assumes the translation component $\tilde { \mathbf { \Delta } } _ { i j }$ of the ijth measurement is corrupted by additive mean-zero isotropic Gaussian noise with concentration parameter $\tau _ { i j } > 0$ , and the rotational component $\tilde { R } _ { i j }$ is corrupted by multiplicative isotropic Langevin noise with mode $\mathbf { I } _ { d }$ and concentration parameter $\kappa _ { i j } \geq 0$

The primary motivation behind our use of the noise model (6.8) (as opposed to the more ‘generic’ exponentiated-Gaussian noise model over general Lie groups mentioned in Chapter 2) is that its associated maximum likelihood estimation takes a particularly simple algebraic form. Indeed, given a set of noisy measurements $\tilde { \mathbf { { T } } } _ { i j }$ sampled from (6.8), a straightforward calculation shows that the associated maximum likelihood estimation is

$$
\begin{array}{c} \text {Problem 6.1 (Pose - Graph Optimization)} \\ p _ {\mathrm{MLE}} ^ {*} = \min_ {\substack {\boldsymbol {t} _ {i} \in \mathbb {R} ^ {d} \\ \boldsymbol {R} _ {i} \in \mathrm{SO} (d) (i, j) \in \overrightarrow {\mathcal {E}}}} \sum_ {(i, j) \in \overrightarrow {\mathcal {E}}} \kappa_ {i j} \| \boldsymbol {R} _ {j} - \boldsymbol {R} _ {i} \tilde {\boldsymbol {R}} _ {i j} \| _ {F} ^ {2} + \tau_ {i j} \left\| \boldsymbol {t} _ {j} - \boldsymbol {t} _ {i} - \boldsymbol {R} _ {i} \tilde {\boldsymbol {t}} _ {i j} \right\| _ {2} ^ {2}. \end{array}\tag{6.9}
$$

In particular, note that the objective appearing in (6.9) is a simple (quadratic) linear least-squares loss. Moreover, the constraints $R _ { i } \ \in \ \mathrm { S O } ( d )$ can be written as quadratic constraints in the planar $( d = 2 )$ and three-dimensional case $( d = 3 )$ . In the next subsection, we will exploit this fact to derive a convex relaxation of Problem 6.1 via Shor’s relaxation.

## 6.1.2.2 Applying Shor’s Relaxation to Pose-Graph Optimization

In this subsection we show how one can apply Shor’s relaxation to derive a convex relaxation of the PGO Problem 6.1.

Simplifying the Maximum Likelihood Estimator. Our first step will be to rewrite Problem 6.1 in a more compact form that only involves rotations, and reveals the correspondence between the optimization problem (6.9) and the underlying graphs $\mathcal { G }$ and $\vec { \mathcal { G } }$ from which it is constructed. To that end, we introduce several matrices constructed from these graphs, and refer the reader to the box on the next page for a primer on (algebraic) graph theory (specifically, the incidence and Laplacian matrices).

## Box 6.1: Elements of Algebraic Graph Theory

Algebraic graph theory studies how to use algebra $( e . g .$ , matrices, vectors) to represent, analyze, and extract information from graphs. Here we review some basic concepts to support the explanations and connections drawn in this chapter. We also point the reader to Chapter 1, which draws connections between graphs and probabilistic graphical models.

A directed graph $\overrightarrow { \mathcal { G } }$ is a pair $( \nu , \mathcal { E } )$ , where $\nu$ is a finite set of nodes, and $\mathcal { E }$ is a set of edges, where each edge contains an ordered pair of nodes. An edge $e \in { \mathcal { E } }$ is in the form $e = ( i , j )$ , meaning that edge $e ,$ incident on nodes i and j, leaves node i and is directed towards node $j$ (i is the tail of the edge, and j is the head).

For a graph with n nodes and m edges, the incidence matrix $\pmb { A } \in \mathbb { R } ^ { m \times n }$ of a directed graph $\vec { \mathcal { G } }$ is a matrix with entries in $\{ - 1 , 0 , + 1 \}$ that describes the structure of the graph. Each row of A corresponds to an edge, and the column corresponding to edge $e = ( i , j )$ has only two non-zero elements, one on the i-th column (equal $\mathrm { t o } - 1 )$ and the other on the $j \mathrm { - t h }$ column (equal to +1). While a more common definition of the incidence matrix has the edges on the columns $( i . e . ,$ the transpose of our $A )$ , here we use this definition to keep some symmetry with the Jacobian matrix in Chapter 1. For instance, the incidence matrix of the graph in Figure 6.2 is:

$$
\boldsymbol {A} = \left[ \begin{array}{l l l l l l l} x _ {1} & x _ {2} & x _ {3} & x _ {4} & x _ {5} & x _ {6} & x _ {7} \\ - 1 & + 1 & & & & & \\ & - 1 & + 1 & & & & \\ & & - 1 & + 1 & & & \\ & & & - 1 & + 1 & & \\ & & & & - 1 & + 1 & \\ & & & & & - 1 & + 1 \\ & + 1 & & & - 1 & & \\ + 1 & & & & & - 1 \end{array} \right] \begin{array}{l} e _ {1 2} \\ e _ {2 3} \\ e _ {3 4} \\ e _ {4 5} \\ e _ {5 6} \\ e _ {6 7} \\ e _ {5 2} \\ e _ {6 1} \end{array}\tag{6.10}
$$

The Laplacian matrix $\ b { L } \in \mathbb { R } ^ { n \times n }$ is defined as $L \triangleq A ^ { \intercal } A$ and also captures the connectivity of the graph. In particular, the i-th diagonal element of L corresponds to the node degree of the i-th node in the graph $( i . e .$ , the number of nodes connected to node i), while an of-diagonal element in position $( i , j )$ is equal to 1 if there is an edge (regardless of its orientation) connecting node i and $j$ or is zero otherwise. For instance, the Laplacian matrix of the

graph in Figure 6.2 is:

$$
\boldsymbol {L} = \left[ \begin{array}{c c c c c c c} x _ {1} & x _ {2} & x _ {3} & x _ {4} & x _ {5} & x _ {6} & x _ {7} \\ + 2 & - 1 & & & & - 1 & \\ - 1 & + 3 & - 1 & & - 1 & & \\ & - 1 & + 2 & - 1 & & & \\ & & - 1 & + 2 & - 1 & & \\ & - 1 & & - 1 & + 3 & - 1 & \\ - 1 & & & & - 1 & + 3 & - 1 \\ & & & & & - 1 & + 1 \end{array} \right] \begin{array}{c} x _ {1} \\ x _ {2} \\ x _ {3} \\ x _ {4} \\ x _ {5} \\ x _ {6} \\ x _ {7} \end{array}\tag{6.11}
$$

We remark that the Laplacian matrix no longer captures the directionality of the edges in the graph since the of-diagonal entries are 1 when there is an edge connecting the corresponding nodes regardless of its orientation.

The smallest eigenvalue of the Laplacian matrix is always equal to zero and the corresponding eigenvector is the vector with all entries equal to one (this can be easily seen from the fact that the entries on each row sum up to zero, hence $\pmb { L } \cdot \mathbf { 1 } = \mathbf { 0 } )$ . It turns out that the number of zero eigenvalues of the Laplacian corresponds to the number of connected components in the graph: a connected graph (where there is path of edges between any pair of nodes, regardless of the orientation of the edges) has a single zero eigenvalue, a graph formed by two disconnected subgraphs has two zero eigenvalues, etc. Moreover, for a connected graph, the second smallest eigenvalue is a measure of how well-connected the graph is, and is also known as the algebraic connectivity or Fiedler value of the graph.

Let us define some key matrices that are related to the graph Laplacian matrix and that will be used in our derivation. We define the translational weight graph $\mathcal { W } ^ { \tau } = ( \mathcal { V } , \mathcal { E } , \{ \tau _ { i j } \} )$ to be the weighted undirected graph with node set $\nu ,$ edge set $\mathcal { E } ,$ and edge weights $\tau _ { i j }$ for $\{ i , j \} \in { \mathcal { E } }$ , and let $\pmb { L } ( \mathcal { W } ^ { \tau } ) \in \mathbb { S } _ { + } ^ { n }$ denote its Laplacian:

$$
\boldsymbol {L} (\mathcal {W} ^ {\tau}) _ {i j} = \left\{ \begin{array}{l l} \sum_ {\{i, k \} \in \mathcal {E}} \tau_ {i k}, & i = j, \\ - \tau_ {i j}, & \{i, j \} \in \mathcal {E}, \\ 0, & \{i, j \} \notin \mathcal {E}. \end{array} \right.\tag{6.12}
$$

This is simply a weighted version of the Laplacian matrix we defined above. Similarly, we write $L ( \tilde { G } ^ { \rho } ) \in \mathbb { S } _ { + } ^ { d n }$ for the connection Laplacian determined by the rotational measurements $\ddot { R } _ { i j }$ and precisions $\kappa _ { i j } ;$ this is the symmetric $( d \times d )$ -block-

structured matrix defined by

$$
\boldsymbol {L} (\tilde {G} ^ {\rho}) _ {i j} \triangleq \left\{ \begin{array}{l l} \Bigl (\sum_ {\{i, k \} \in \mathcal {E}} \kappa_ {i k} \Bigr) \mathbf {I} _ {d}, & i = j, \\ - \kappa_ {i j} \tilde {\boldsymbol {R}} _ {i j}, & (i, j) \in \overrightarrow {\mathcal {E}}, \\ - \kappa_ {j i} \tilde {\boldsymbol {R}} _ {j i} ^ {\intercal}, & (j, i) \in \overrightarrow {\mathcal {E}}, \\ \mathbf {0} _ {d \times d}, & \{i, j \} \notin \mathcal {E}. \end{array} \right.\tag{6.13}
$$

We also define a few matrices constructed from the set of translation observations $\tilde { t } _ { i j }$ . We let $\tilde { V } \in \mathbb { R } ^ { n \times d n }$ be the $( 1 \times d )$ -block-structured matrix with $( i , j ) -$ blocks determined by

$$
\tilde {\boldsymbol {V}} _ {i j} \triangleq \left\{ \begin{array}{l l} \sum_ {\{k \in \mathcal {V} | (j, k) \in \overrightarrow {\mathcal {E}} \}} \tau_ {j k} \widetilde {\boldsymbol {t}} _ {j k} ^ {\mathsf {T}}, & i = j, \\ - \tau_ {j i} \widetilde {\boldsymbol {t}} _ {j i} ^ {\mathsf {T}}, & (j, i) \in \overrightarrow {\mathcal {E}}, \\ 0 _ {1 \times d}, & \text {otherwise}, \end{array} \right.\tag{6.14}
$$

Let $\tilde { D } \in \mathbb { R } ^ { m \times d n }$ be the $( 1 \times d )$ -block-structured matrix with rows and columns indexed by $e \in { \vec { \mathcal { E } } }$ and $k \in \mathcal V$ , respectively, and whose $( e , k )$ -block is given by

$$
\tilde {\boldsymbol {D}} _ {e k} \triangleq \left\{ \begin{array}{l l} - \widetilde {\boldsymbol {t}} _ {k j} ^ {\mathsf {T}}, & e = (k, j) \in \overrightarrow {\mathcal {E}}, \\ \boldsymbol {0} _ {1 \times d}, & \text { otherwise }, \end{array} \right.\tag{6.15}
$$

and $\Omega \triangleq \operatorname { D i a g } ( \tau _ { e _ { 1 } } , \dots , \tau _ { e _ { m } } ) \in \mathbb { S } ^ { m }$ denote the diagonal matrix constructed from the translation measurement precisions. Finally, we also aggregate the rotation and translation state estimates into the block matrices $R \triangleq { \big ( } R _ { 1 } \quad \cdot \cdot \cdot \mathbf { \nabla } R _ { n } { \big ) } \in$ ${ \mathrm { S O } } ( d ) ^ { n } \subset \mathbb { R } ^ { d \times d n }$ and $\pmb { t } \triangleq \left( \pmb { t } _ { 1 } \quad \ldots \quad \pmb { t } _ { n } \right) \in \mathbb { R } ^ { d n }$

With these definitions in hand, let us return to Problem 6.1. Observe that if we $\mathit { f i x }$ a value of the rotational states $\pmb { R } _ { 1 } , \ldots , \pmb { R } _ { n }$ , problem (6.9) reduces to a linear leastsquares problem in the remaining translational decision variables $\boldsymbol { t } _ { 1 } , \ldots , \boldsymbol { t } _ { n } \in \mathbb { R } ^ { d }$ Consequently, we may solve for an optimal assignment ${ \pmb t } ^ { * } ( R )$ of the translational states as functions of the rotational states $\mathbf { \nabla } _ { R } .$

$$
\boldsymbol {t} ^ {*} (\boldsymbol {R}) = - \operatorname{vec} \left(\boldsymbol {R} ^ {*} \tilde {\boldsymbol {V}} ^ {\top} \boldsymbol {L} \left(\mathcal {W} ^ {\tau}\right) ^ {\dagger}\right).\tag{6.16}
$$

By substituting the optimal assignment (6.16) into (6.9), we may thus analytically eliminate the translational states from the pose-graph SLAM MLE, producing the following simplified (but equivalent) problem involving only the rotational states:

Problem 6.2 (Rotation-Only Pose-Graph Optimization)

$$
p _ {\mathrm{MLE}} ^ {*} = \min _ {\boldsymbol {R} \in \mathrm{SO} (d) ^ {n}} \operatorname{tr} \left(\tilde {\boldsymbol {Q}} \boldsymbol {R} ^ {\mathsf {T}} \boldsymbol {R}\right)\tag{6.17a}
$$

$$
\tilde {\boldsymbol {Q}} = \boldsymbol {L} (\tilde {G} ^ {\rho}) + \tilde {\boldsymbol {D}} ^ {\mathsf {T}} \boldsymbol {\Omega} ^ {\frac {1}{2}} \boldsymbol {\Pi} \boldsymbol {\Omega} ^ {\frac {1}{2}} \tilde {\boldsymbol {D}},\tag{6.17b}
$$

where $\textbf { \textit { \textbf { I I } } } \in \mathbf { \textit { \textbf { R } } }$ <sup>m</sup>×<sup>m</sup> is the matrix of the orthogonal projection $\pi \colon \mathbb { R } ^ { m } . $ $\ker ( A ( { \overrightarrow { \mathcal { G } } } ) \Omega ^ { \frac { 1 } { 2 } } )$ onto the kernel of the weighted incidence matrix $A ( \overrightarrow { \mathcal { G } } ) \Omega ^ { \frac { 1 } { 2 } }$ of $\overrightarrow { \mathcal { G } }$ .

We observe that (6.17) involves only n rotation matrices (rather than n poses) and the problem now resembles the standard problem of multiple rotation averaging [435] (but with a more involved expression for the data matrix $\tilde { Q } )$ . On the more technical side, we note that although Π is generically dense, by exploiting the fact that it is derived from the graph $\overrightarrow { \mathcal { G } }$ , one can show that it admits the decomposition:

$$
\boldsymbol {\Pi} = \mathbf {I} _ {m} - \boldsymbol {\Omega} ^ {\frac {1}{2}} \bar {\boldsymbol {A}} (\overrightarrow {\mathcal {G}}) ^ {\top} \boldsymbol {L} ^ {- \top} \boldsymbol {L} ^ {- 1} \bar {\boldsymbol {A}} (\overrightarrow {\mathcal {G}}) \boldsymbol {\Omega} ^ {\frac {1}{2}}\tag{6.18}
$$

where $\bar { A } ( \vec { \mathcal { G } } ) \Omega ^ { \frac { 1 } { 2 } } = L Q$ <sub>1</sub> is a thin LQ decomposition of $\bar { A } ( \vec { \mathcal { G } } ) \Omega ^ { \frac { 1 } { 2 } }$ and $\bar { A } ( \vec { \mathcal { G } } )$ is the reduced incidence matrix of $\overrightarrow { \mathcal { G } }$ obtained by deleting one of $A ( { \overrightarrow { \mathcal { G } } } )$ ’s rows. Note that expression (6.18) requires only the lower-triangular factor $L ,$ which will be sparse whenever the underlying graph $\vec { \mathcal { G } }$ is, and can be obtained eficiently in practice. The sparse decomposition (6.17b)–(6.18) of the data matrix $\tilde { Q }$ will play a critical role in our implementation of eficient optimization methods $( c f .$ Section 6.1.2.3).

Forming the Relaxation. Now we derive the semidefinite relaxation of Problem 6.1 that we will solve in practice, taking advantage of the simplified form (6.17).

We begin by relaxing the condition $\pmb { { \cal R } } \in \mathrm { S O } ( d ) ^ { n }$ to $\pmb { { \cal R } } \in \mathrm { O } ( d ) ^ { n }$ . The advantage of the latter condition versus the former is that since orthogonal matrices are defined by a set of (quadratic) orthonormality constraints, the orthogonal relaxation of Problem 6.2 is a homogeneous QCQP. Indeed, writing

$$
\begin{array}{c} \text {BlockDiag} _ {d \times d} \colon \mathbb {R} ^ {d n \times d n} \to \mathbb {R} ^ {d \times d n} \\ \text {BlockDiag} _ {d \times d} (M) \triangleq (M _ {1 1}, \ldots , M _ {n n}) \end{array}\tag{6.19}
$$

for the linear map that extracts the n diagonal blocks of a $( d \times d )$ -block-structured matrix M, we may expresss the orthogonal relaxation of (6.17) in an extrinsicallyconstrained form as

$$
p _ {O} ^ {*} = \min _ {\boldsymbol {R} \in \mathbb {R} ^ {d \times d n}} \operatorname{tr} \left(\tilde {\boldsymbol {Q}} \boldsymbol {R} ^ {\intercal} \boldsymbol {R}\right) \quad \text {s.t.} \quad \operatorname{BlockDiag} _ {d \times d} (\boldsymbol {R} ^ {\intercal} \boldsymbol {R}) = (\mathbf {I} _ {d}, \dots , \mathbf {I} _ {d}).\tag{6.20}
$$

Now we note that (6.20) has a structure similar to (QCQP), but where the vector x is replaced with a matrix R. While in (QCQP) we obtained a relaxation by replacing terms $\mathbf { \pmb { x } } \mathbf { \pmb { x } } ^ { \mathsf { T } }$ with a suitable rank-1 matrix X and dropping the rank-1 constraint, we can similarly replace the terms $\scriptstyle { R ^ { \intercal } R }$ in (6.20) with a rank-d matrix $z$ and drop the rank constraint, thus obtaining the following semidefinite relaxation of the simplified PGO problem (6.17):

Problem 6.3 (Semidefinite Relaxation for Pose-Graph Optimization)

$$
p _ {\mathrm{SDP}} ^ {*} = \min _ {\boldsymbol {Z} \in \mathbb {S} _ {+} ^ {d n}} \operatorname{tr} \left(\tilde {\boldsymbol {Q}} \boldsymbol {Z}\right) \quad \text {s.t.} \quad \operatorname{BlockDiag} _ {d \times d} (\boldsymbol {Z}) = (\mathbf {I} _ {d}, \ldots , \mathbf {I} _ {d}).\tag{6.21}
$$

As we saw in Section 6.1.1, this construction immediately implies that $p _ { \mathrm { M L E } } ^ { * } \geq$ $p _ { O } ^ { * } \geq p _ { \mathrm { S D P } } ^ { * }$ . Moreover, if after solving the SDP relaxation (6.21), it so happens that the recovered minimizer $Z ^ { \ast } \in \mathbb { S } _ { + } ^ { d n }$ admits a rank-d factorization of the form $z ^ { * } =$ $\mathbf { \mathbf { \mathbf { { R } } ^ { * } \mathbf { \mathbf { \mathbf { \mathbf { \Lambda } } } } } } \mathbf { \mathbf { \mathbf { { R } } ^ { * } } }$ with $R ^ { * } \in \mathrm { S O } ( d ) ^ { n }$ , then $\pmb { R } ^ { * }$ will itself be a globally optimal solution of the

PGO Problem 6.2. The remarkable fact that justifies our interest in the relaxation (6.21) is that this favorable situation actually occurs in practice. Specifically, we have the following theorem (see [937] for a proof):

Theorem 6.1 (Exact Recovery of PGO Solutions from Problem 6.3) Let $\bar { Q }$ be the matrix of the form $( 6 . 1 7 b )$ constructed using the ground-truth relative transforms $\bar { T } _ { i j }$ in $( 6 . 8 )$ . There exists a constant $\beta \triangleq \beta ( \bar { Q } ) > 0$ (depending upon $\bar { Q } )$ such that, i $f \| \tilde { Q } - \bar { Q } \| _ { 2 } < \beta$ , then:

1 The semidefinite relaxation Problem 6.3 has a unique solution $Z ^ { * }$ , and

2 $Z ^ { * } = R ^ { * } { } ^ { \mathsf { T } } R ^ { * }$ , where $R ^ { * } \in \mathrm { S O } ( d ) ^ { n }$ is a minimizer of the maximum likelihood estimation Problem 6.2.

In brief, Theorem 6.1 guarantees that as long as the noise corrupting the available measurements $\tilde { \pmb { T } } _ { i j }$ is not too large, we can recover a global minimizer $\pmb { R } ^ { * }$ of Problem 6.2 (and thus also a global minimizer $( R ^ { * } , t ^ { * } )$ of Problem 6.1 via (6.16)) by solving the SDP relaxation (6.21).

## 6.1.2.3 Eficiently Solving the Relaxation via the Riemannian Staircase

As a semidefinite program, Problem 6.3 can in principle be solved eficiently, $i . e .$ in polynomial time. However, in practice the high computational cost of storing and manipulating the dense matrix decision variable $z$ appearing in (6.21) prevents general-purpose interior-point methods from scaling efectively to problems in which the dimension of Z is greater than a few thousand. Unfortunately, the instances of (6.21) arising in robotics and computer vision applications are typically one to two orders of magnitude larger than this maximum efective problem size, placing them well beyond the reach of general-purpose techniques. Consequently, in this subsection we develop a specialized, structure-exploiting optimization procedure that is capable of solving large-scale instances of Problem 6.3 eficiently.

Exploiting Low-rank Structure. The main idea behind our approach is to exploit the existence of low-rank solutions to Problem 6.3. Specifically, note that while the decision variable Z appearing in the relaxation (6.21) is a generic highdimensional PSD matrix, Theorem 6.1 guarantees that the solution $Z ^ { * }$ we seek admits a very concise description in the factored form $Z ^ { * } = R ^ { * } { } ^ { \mathsf { T } } R ^ { * }$ whenever exactness obtains. Moreover, it turns out that even when exactness fails to hold, minimizers of Problem 6.3 typically have a rank r not much greater than $d ,$ and therefore also admit a concise symmetric rank decomposition of the form $z ^ { * } =$ $Y ^ { * \mathsf { T } } \mathbf { Y } ^ { * }$ for some $Y ^ { * } \in \mathbb { R } ^ { r \times d n }$

In their seminal work, Burer and Monteiro [125, 124] proposed an elegant general approach to exploit the existence of such low-rank solutions: simply replace every instance of the decision variable $z$ in (6.21) with a symmetric rank-r factorization of the form $\pmb { Y } ^ { \top } \pmb { Y }$ (for some $\pmb { Y } \in \mathbb { R } ^ { r \times d n } )$ to produce the following Burer-Monteiro

factorization of (6.21):

$$
p _ {\mathrm{SDPLR}} ^ {*} (r) = \min _ {\boldsymbol {Y} \in \mathbb {R} ^ {r \times d n}} \operatorname{tr} \left(\tilde {\boldsymbol {Q}} \boldsymbol {Y} ^ {\top} \boldsymbol {Y}\right) \quad \text {s.t.} \quad \operatorname{BlockDiag} _ {d \times d} (\boldsymbol {Y} ^ {\top} \boldsymbol {Y}) = (\mathbf {I} _ {d}, \ldots , \mathbf {I} _ {d}).\tag{6.22}
$$

We remark that since $\mathbf { \nabla } _ { \mathbf { Y } } \mathsf { T } _ { \mathbf { Y } }$ is symmetric and PSD by construction, in (6.22) it is no longer necessary to explicitly enforce the positive-semidefiniteness constraint from the original SDP (6.21). Moreover, we note that this problem is strikingly similar to the problem we relaxed (6.20), with the important distinction that now the matrix Y has size $r \times$ dn instead of $d \times d n$ , with $r > d ;$ in other words, (6.22) reformulates the problem in a higher-dimensional space compared to (6.20).

If the maximum rank parameter r in (6.22) is chosen to be “small” $( i . e . , r \ll d n )$ then dim $( \mathbb { R } ^ { r \times d n } ) = r n d \ll ( d n + 1 ) d n / 2 = \dim ( \mathbb { S } _ { + } ^ { d n } )$ ; that is, the search space for (6.22) is much lower-dimensional than the search space for (6.21). Consequently, Burer and Monteiro proposed to apply fast nonlinear programming algorithms to the low-dimensional NLP (6.22) in order to search for a low-rank factor $\pmb { Y } ^ { * } \in \mathbb { R } ^ { r \times }$ dn of a minimizer $Z ^ { * } = Y ^ { * } { } ^ { \mathsf { T } } Y ^ { * }$ of the original SDP (6.21).

Exploiting Geometric Structure. Note that if we additionally partition Y into $r \times d$ blocks as $\pmb { Y } = ( \pmb { Y } _ { 1 } , \dots , \pmb { Y } _ { n } ) \in \mathbb { R } ^ { r \times d n }$ , then the block-diagonal constraints appearing in (6.22) are equivalent to $Y _ { i } ^ { \mathsf { T } } Y _ { i } = \mathbf { I } _ { d }$ for all $i \in [ n ]$ ; geometrically, this condition states that the columns of each block $\pmb { Y } _ { i } \in \mathbb { R } ^ { r \times d }$ form an orthonormal frame. In general, the set of all orthonormal k-frames in $\mathbb { R } ^ { p }$ ,

$$
\operatorname{St} (k, p) \triangleq \left\{\boldsymbol {Y} \in \mathbb {R} ^ {p \times k} \mid \boldsymbol {Y} ^ {\top} \boldsymbol {Y} = \mathbf {I} _ {k} \right\},\tag{6.23}
$$

forms a smooth compact matrix manifold, called the Stiefel manifold. This implies that the equality-constrained nonlinear program (6.22) is equivalent to the following unconstrained optimization problem defined on a product of Stiefel manifolds:

Problem 6.4 (Burer-Monteiro-factored SDP relaxation as manifold optimization)

$$
p _ {\mathrm{SDPLR}} ^ {*} (r) = \min _ {\boldsymbol {Y} \in \operatorname{St} (d, r) ^ {n}} \operatorname{tr} \left(\tilde {\boldsymbol {Q}} \boldsymbol {Y} ^ {\top} \boldsymbol {Y}\right).\tag{6.24}
$$

While formulations (6.22) and (6.24) are equivalent, the latter provides important computational advantages. In particular, recognizing that the feasible set is a product of Stiefel manifolds enables us to apply specialized algorithms for optimization over smooth manifolds, which are substantially simpler, faster, and more accurate than general-purpose equality-constrained nonlinear programming techniques [106].

Ensuring Global Optimality. While the reduction from Problem 6.3 to Problem 6.4 dramatically reduces the size of the optimization problem that needs to be solved, it comes at the expense of (re)introducing the quadratic orthonormality constraints (6.23), which are nonconvex. It may therefore not be clear whether anything has really been gained by relaxing Problem 6.2 to Problem 6.4, since it appears that we may have simply replaced one dificult nonconvex optimization problem with another. The following remarkable result (adapted from Boumal et al. [107]) justifies this approach:

Theorem 6.2 (A Suficient Condition for Global Optimality in Problem 6.4) If $Y \in { \mathrm { S t } } ( d , r ) ^ { n }$ is a (row) rank-deficient second-order critical point of Problem $6 . 4 ,$ then Y is a global minimizer of Problem $6 . 4$ and $Z ^ { * } = Y ^ { \mathsf { T } } Y$ is a solution of the semidefinite relaxation Problem 6.3.

Theorem 6.2 immediately suggests a simple procedure, the Riemannian Staircase, for recovering solutions $Z ^ { * }$ of Problem 6.3 by applying fast local optimization algorithms to a sequence of instances of Problem 6.4. In brief, starting at some (small) initial maximum rank $r \geq d ,$ we apply a local solver to (6.24) (more precisely, a second-order Riemannian optimization algorithm) to recover a second-order critical point $Y ^ { * } \in { \mathrm { S t } } ( d , r ) ^ { n }$ . If $Y ^ { * }$ is rank-deficient, then Theorem 6.2 proves that $Y ^ { * }$ is a global minimizer of (6.24), and $Z ^ { * } = Y ^ { * } { } ^ { \top } Y ^ { * }$ is a solution of (6.21). On the other hand, if $Y ^ { * }$ is not rank deficient, we can simply increase the maximum rank r and try again. Note that since every $\pmb { Y } \in \mathbb { R } ^ { r \times d n }$ is (row) rank-deficient for $r \geq d n + 1$ , the Riemannian Staircase is guaranteed to terminate with an optimal solution $Y ^ { * }$ after finitely many iterations. However, typically only one or two “stairs” sufice.

Finally, we remark that from a practical standpoint, the Riemannian Staircase functions as a lightweight meta-algorithm that “wraps around” the same class of fast (second-order) local optimization algorithms commonly applied to SLAM problems in practice. This approach thus enables us to preserve the speed of current state-ofthe-art SLAM techniques while additionally guaranteeing the recovery of globally optimal solutions, thereby achieving the best of both worlds.

## 6.1.2.4 Rounding the Solution

We have just seen that the Riemannian Staircase provides an eficient algorithm for recovering a low-rank factor $Y ^ { * } \in \mathrm { S t } ( d , r ) ^ { n }$ of a solution $Z ^ { * } = Y ^ { * } { } ^ { \mathsf { T } } Y ^ { * }$ of the relaxation Problem 6.3. However, ideally we would like to extract an optimal solution $R ^ { * } \in \mathrm { S O } ( d ) ^ { n }$ of the PGO Problem 6.2 from $Z ^ { * }$ whenever the relaxation (6.21) is exact, and a feasible approximate solution $\hat { R } \in \mathrm { S O } ( d ) ^ { n }$ otherwise. In this subsection, we describe an eficient rounding procedure that accomplishes these aims by operating directly on the low-rank factor ${ \cal Y } ^ { \ast } \left( i . e . \right.$ , without the need to explicitly construct the dense, high-dimensional matrix $Z ^ { * } )$

The main insight that underpins our approach is that, in the event that the relaxation (6.21) is exact, $R ^ { * } , Z ^ { * }$ , and $Y ^ { * }$ will satisfy the relation

$$
\boldsymbol {Y} ^ {* \top} \boldsymbol {Y} ^ {*} = \boldsymbol {Z} ^ {*} = \boldsymbol {R} ^ {* \top} \boldsymbol {R} ^ {*}.\tag{6.25}
$$

In this case, equation (6.25) implies that the low-rank factor $\pmb { Y } ^ { * } \in \mathbb { R } ^ { r \times d n }$ actually has rank $d ,$ and consequently that $\pmb { R } ^ { * }$ can be recovered from $Y ^ { * }$ by computing a thin singular value decomposition of the latter. More generally, in the event that (6.21) is not exact, we can still recover an optimal rank-d approximation $\hat { \pmb { R } } \in \mathbb { R } ^ { d \times d n } \ \mathrm { o f } \ Y ^ { * }$ using a truncated singular value decomposition, and then project the individual d d blocks $\hat { R } _ { i }$ of $\hat { R }$ onto SO(d) (again using an SVD) to produce a feasible approximate solution of Problem 6.2.

<div class="mineru-algorithm" style="white-space: pre-wrap; font-family:monospace;">
Algorithm 3 The SE-Sync algorithm

Input: An initial point  $Y \in \text{St}(d, r_0)^n$ ,  $r_0 \geq d + 1$ .

Output: A feasible estimate  $\hat{T} \in \text{SE}(d)^n$  for the maximum likelihood estimation Problem 6.1, and the lower bound  $p_{SDP}^*$  for Problem 6.1's optimal value.

1: function SE-SYNC(Y)

2: Set  $Y^* \leftarrow RIEMANNIANSTAIRCASE(Y)$ .

3: Set  $p_{SDP}^* \leftarrow F(\hat{Q}Y^{*T}Y^*)$ .

4: Set  $\hat{R} \leftarrow ROUND SOLUTION(Y^*)$ .

5: Recover the optimal translational estimates  $\hat{t}$  corresponding to  $\hat{R}$  via (6.16).

6: Set  $\hat{T} \leftarrow (\hat{t}, \hat{R})$ .

7: return  $\left\{\hat{T}, p_{SDP}^*\right\}$ 

8: end function
</div>

## 6.1.2.5 SE-Sync: The Complete Algorithm

Combining the eficient SDP optimization approach of Section 6.1.2.3 with the rounding procedure of Section 6.1.2.4 produces $S E { \mathrm { - } } S y n c$ (Algorithm 3), our certifiably correct algorithm for pose-graph optimization [937].

When applied to an instance of PGO, SE-Sync returns a feasible point $\hat { \textbf { \textit { T } } } \in$ $\operatorname { S E } ( d ) ^ { n }$ of the maximum likelihood estimation Problem 6.1 together with a lower bound $p _ { \mathrm { S D P } } ^ { * } \leq p _ { \mathrm { M L E } } ^ { * }$ on its optimal value. This lower bound in turn provides an upper bound on the suboptimality of any feasible point $\pmb { T } = ( \pmb { t } , \pmb { R } ) \in \mathrm { S E } ( d ) ^ { n }$ as a solution of Problem 6.1 according to

$$
F (\tilde {\boldsymbol {Q}} \boldsymbol {R} ^ {\mathsf {T}} \boldsymbol {R}) - p _ {\mathrm{SDP}} ^ {*} \geq F (\tilde {\boldsymbol {Q}} \boldsymbol {R} ^ {\mathsf {T}} \boldsymbol {R}) - p _ {\mathrm{MLE}} ^ {*}.\tag{6.26}
$$

Moreover, in the case that the relaxation in Problem 6.3 is exact, the estimate ${ \hat { \pmb { T } } } = ( { \hat { \pmb { t } } } , { \hat { \pmb { R } } } ) \in \mathrm { S E } ( d ) ^ { n }$ returned by Algorithm 3 attains this lower bound:

$$
F (\tilde {\boldsymbol {Q}} \hat {\boldsymbol {R}} ^ {\mathsf {T}} \hat {\boldsymbol {R}}) = p _ {\mathrm{SDP}} ^ {*}.\tag{6.27}
$$

Consequently, verifying a posteriori that (6.27) holds provides a computational certificate of $\hat { \pmb { T } } \mathrm { ^ { * } s }$ correctness as a solution of Problem 6.1. SE-Sync is thus a certifiably correct algorithm for pose-graph optimization, as claimed.

Sample certifiably optimal results obtained with SE-Sync are shown in Figure 6.3 and discussed in depth in [937]. The paper [937] also reports a runtime analysis showing that the algorithm can be as fast if not faster than traditional local solvers.

![](images/7a5a9571709f218172aae8ff0fc3c9a0a0b3ee53daae4641c146fb07706a602b.jpg)  
(a) sphere

![](images/aaf1f6d4699901277815ac984a83f7c880438b7121a49c040747f5c4dcba30a4.jpg)  
(b) torus

![](images/6bfb07c7d6120b7734a3a8fed4d179c48b86264a1ae5f1c916e7c4a1a7418076.jpg)  
(c) grid

![](images/6790fd8f3bdfdff3b31fe7848d1299c590f63f31edf61e3951c97bdf67ef381a.jpg)  
(d) garage

![](images/ade440c3c94d3284ecb2d400f9b0aa8e310b0d6837225d25ba5ec41238270015.jpg)  
(e) cubicle

![](images/cb77664d0afcc4bad38680d357dbb6bfdc7e150381e561bb95b8116f7fcb528b.jpg)  
(f) rim  
Figure 6.3 Globally optimal solutions for pose-graph optimization benchmarking datasets (From [937]).

## 6.1.3 Landmark-based SLAM

While in the previous section we have shown how to obtain a fast certifiable algorithm for PGO, in this section we show that the same derivation can be extended to landmark-based SLAM, specifically for the case where the robot takes bearing and range (i.e., relative position) measurements to landmarks.

Let $\pmb { m } _ { n + 1 } , \ldots , \pmb { m } _ { n + \ell } \in \mathbb { R } ^ { d }$ be the set of ℓ landmark positions that we wish to estimate in addition to the n robot poses. It turns out that these new landmark variables can be easily integrated into the existing implementation of SE-Sync. To do this, we treat the map point variables as pure translation variables (i.e., poses without a rotational component).

We assume that we have a set of $N _ { m }$ landmark measurements, $\left\{ \tilde { m } _ { i k } \right\}$ , describing the relative position of landmark $\mathbf { \nabla } m _ { k }$ with respect to the i-th pose of the robot. We also assume that these measurements are corrupted by additive zero-mean isotropic Gaussian noise —the same form as the translation measurements in (6.8):

$$
\tilde {\boldsymbol {m}} _ {i k} = \bar {\boldsymbol {m}} _ {i k} + \boldsymbol {m} _ {i k} ^ {\epsilon}, \quad \boldsymbol {m} _ {i j} ^ {\epsilon} \sim \mathcal {N} \left(0, \mu_ {i j} ^ {- 1} I _ {d}\right).\tag{6.28}
$$

To track these new measurements, we augment the pose-graph, $\vec { \mathcal { G } }$ , with a new set of vertices, $\nu _ { m } .$ , and edges, $\vec { \mathcal { E } } _ { m } .$ , with a one-to-one correspondence to the map variables and landmark measurements, respectively. Moreover, we denote with $\overrightarrow { \mathcal { E } } _ { r }$ the edges corresponding to relative pose measurements of the robot pose $( e . g .$ , odometry), which are assumed to follow the same measurement model as in (6.8).

The maximum likelihood estimation problem given the landmark measurements and the relative pose measurements becomes:

$$
\begin{array}{l} \text {lem 6.5 (Landmark - based SLAM)} \\ \min _ {\substack {\boldsymbol {R} _ {i} \in \mathrm{SO} (d) \\ \boldsymbol {t} _ {i} \in \mathbb {R} ^ {d}, \boldsymbol {m} _ {k} \in \mathbb {R} ^ {d}}} \sum_ {(i, j) \in \overrightarrow {\mathcal {E}} _ {r}} \kappa_ {i j} \| \boldsymbol {R} _ {j} - \boldsymbol {R} _ {i} \tilde {\boldsymbol {R}} _ {i j} \| _ {F} ^ {2} + \tau_ {i j} \left\| \boldsymbol {t} _ {j} - \boldsymbol {t} _ {i} - \boldsymbol {R} _ {i} \tilde {\boldsymbol {t}} _ {i j} \right\| _ {2} ^ {2} \\ \qquad + \sum_ {(i, k) \in \vec {\mathcal {E}} _ {m}} \mu_ {i k} \| \boldsymbol {m} _ {k} - \boldsymbol {t} _ {i} - \boldsymbol {R} _ {i} \tilde {\boldsymbol {m}} _ {i k} \| _ {2} ^ {2}. \end{array}\tag{6.29}
$$

It turns out that the remainder of the development we presented in the previous section can be used as is, if we treat the map variables as if they were pose translations. The optimal rotation variables can be found by applying the SE-Sync algorithm with a cost matrix $\tilde { Q }$ that has been modified to include the efect of the map variables. In particular, the weight matrix Ω and measurement matrix $\tilde { D }$ are redefined as follows:

$$
\begin{array}{c} \boldsymbol {\Omega} = \text {BlockDiag} (\boldsymbol {\Omega} _ {\tau}, \boldsymbol {\Omega} _ {\mu}), \\ \boldsymbol {\Omega} _ {\tau} = \text {Diag} (\tau_ {1}, \ldots , \tau_ {N _ {p}}), \\ \boldsymbol {\Omega} _ {\mu} = \text {Diag} (\mu_ {1}, \ldots , \mu_ {N _ {m}}), \end{array} \quad \tilde {\boldsymbol {D}} _ {e k} = \left\{ \begin{array}{l l} - \tilde {\boldsymbol {t}} _ {k j} ^ {T}, & e = (k, j) \in \overrightarrow {\mathcal {E}} _ {r}, \\ - \tilde {\boldsymbol {m}} _ {k j} ^ {T}, & e = (k, j) \in \overrightarrow {\mathcal {E}} _ {m}, \\ \mathbf {0} _ {1 \times d}, & \text {otherwise} \end{array} \right.\tag{6.30}
$$

Additionally, the projection matrix Π is modified to account for the form of the new measurement graph, $\overrightarrow { \mathcal { G } } = ( \mathcal { V } \cup \mathcal { V } _ { m } , \overrightarrow { \mathcal { E } } _ { \tiny r } \cup \overrightarrow { \mathcal { E } } _ { \tiny m } )$ . Note that the connection Laplacian $L ( { \tilde { G } } ^ { \rho } )$ remains unafected by the new map measurements.

Once the optimal rotations have been found, both the pose translations and the map points can be recovered in closed form similarly to (6.16):

$$
\left[ \begin{array}{c c} \boldsymbol {t} ^ {* T} & \boldsymbol {m} ^ {* T} \end{array} \right] ^ {T} = - \mathrm{vec} \left(\boldsymbol {R} ^ {*} \tilde {\boldsymbol {V}} ^ {T} \boldsymbol {L} (\mathcal {W} ^ {\tau}) ^ {\dagger}\right),\tag{6.31}
$$

where we have assumed that the pose translation variables and map points have been ordered appropriately and $\tilde { V }$ and $L ( \mathcal { W } ^ { \tau } ) ^ { \dagger }$ have also been updated to include the map point measurements similarly to (6.30).

It is often the case in typical SLAM problems that there are many more map points than poses. As such, when including map variables in the SE-Sync formulation, it is important to ensure the algorithm remains eficient with respect to the number of map points in the problem. In the case where the number of map points is large, the bottleneck of the SE-Sync algorithm becomes the formation of the cost matrix $\tilde { Q }$ . It has been shown that the classical Schur-complement trick can be used to ensure that this matrix can be constructed with time complexity that is linear in the number of landmarks [465]. This linear dependence can be seen clearly in Figure $6 . 4 ( \mathrm { b } )$ , where it can also be seen that the computational cost of other components of SE-Sync algorithm does not increase with the number of landmarks.

Finally, the inclusion of the landmarks in the formulation has been shown to afect the exactness of the SDP relaxation. In particular, increasing the number of landmarks and the connectivity of the pose-to-map measurement graph have been shown to improve the exactness of the relaxation. More specifically, they increase the noise level for which a given problem has an exact relaxation. A demonstration of the efect of these parameters on a simple SLAM problem is shown in Figure 6.4(c).

![](images/44d924e1d39490d20a5e7d1d54d3f0c0c067b9aa7c4123e7b6a4b158c867631c.jpg)

![](images/cd35af39aa4b4d1ba25000d9681025c961cfb26ec04d98bea7d7f74dc5aafa40.jpg)  
Figure 6.4 (a) An example of a local and global minimum for a simple landmark-based SLAM problem. (b) Runtime for the example shown in (a). Runtime increases linearly with respect to the size of the map and the bottleneck is the construction of the data matrix, Q<sup>˜</sup> (shown in blue). (c) A study of the exactness of the relaxation via corank of a certificate matrix (exact when this metric is three). The noise on the measurements was set to a baseline standard deviation —0.866 meters for translation and 0.573 degrees for rotation— scaled by the “Noise Level” multiplier indicated in the plots. In general, the noise level for which the problem remains exact increases as the number of map points and the pose-to-map connectivity increase. From [465] (©2023 IEEE).

## 6.1.4 Extensions: Range Measurements, Anisotropic Noise, and Outliers

This section shows that the machinery presented above (i.e., Shor’s relaxation and the Riemannian Staircase solver) can be extended to other SLAM problems (Section 6.1.4.1). Moreover, we discuss more general tools to obtain semidefinite relaxations (which can be understood as a generalization of Shor’s relaxation) that further expand the set of certifiable algorithms for SLAM, but create additional challenges when solving the resulting SDP (Section 6.1.4.2).

## 6.1.4.1 A Fast Certifiable Algorithm for Range-Aided SLAM

In the previous section, we showed how to develop certifiable algorithms for SLAM problems where the measurements are relative positions and relative rotations. Here we show that the same approach can be applied to problems involving range measurements, following the results presented in [840]. Specifically, we assume that we can measure distances $\tilde { r } _ { i j }$ between variables i and $j , e . g .$ , the distance between two robot poses or between a landmark and a robot pose. The optimization problem for range-aided SLAM can be stated as follows.

$$
\begin{array}{l} \text {Problem 6.6 (Range - Aided SLAM)} \\ \min_ {\substack {\boldsymbol {R} _ {i} \in \mathrm{SO} (d) \\ \boldsymbol {t} _ {i} \in \mathbb {R} ^ {d}}} \sum_ {(i, j) \in \overrightarrow {\mathcal {E}}} \kappa_ {i j} \| \boldsymbol {R} _ {j} - \boldsymbol {R} _ {i} \tilde {\boldsymbol {R}} _ {i j} \| _ {F} ^ {2} + \tau_ {i j} \left\| \boldsymbol {t} _ {j} - \boldsymbol {t} _ {i} - \boldsymbol {R} _ {i} \tilde {\boldsymbol {t}} _ {i j} \right\| _ {2} ^ {2} \\ \qquad + \sum_ {(i, j) \in \overrightarrow {\mathcal {E}} _ {d}} \gamma_ {i j} \left(\| \boldsymbol {t} _ {j} - \boldsymbol {t} _ {i} \| - \tilde {r} _ {i j}\right) ^ {2} \end{array}\tag{6.32}
$$

The first line in the objective function of Problem 6.6 is the same we have used in the previous sections: these are relative pose measurements, $e . g .$ , corresponding to the robot odometry. The diference in this problem is the second line, which contains terms corresponding to the distance measurements $\tilde { r } _ { i j }$ (the set $\overrightarrow { \mathcal { E } } _ { d }$ is the set of pairs $( i , j )$ such that a range measurement is available), weighted by the inverse variance $\gamma _ { i j }$ of these measurements. The range-only measurements $\tilde { r } _ { i j }$ are distinct in that they provide information about the relative distance between poses, i and $j ,$ but no information about the bearing or relative orientation. Problem 6.6 can be used to model a variety of practical SLAM problems, from landmark-based SLAM with distance measurements to landmarks, to multi-robot SLAM problems where the robots take relative range measurements.<sup>5</sup>

The challenge in applying Shor’s relaxation to Problem 6.6 is that (6.32) is not a QCQP: expanding the square $\begin{array} { r } { \big ( \| \pmb { t } _ { j } - \pmb { t } _ { i } \| - \widetilde { r } _ { i j } \big ) ^ { 2 } = \| \pmb { t } _ { j } - \pmb { t } _ { i } \| ^ { 2 } + 2 \widetilde { r } _ { i j } \| \pmb { t } _ { j } - \pmb { t } _ { i } \| + \widetilde { r } _ { i j } ^ { 2 } } \end{array}$ reveals that this expression is not quadratic in the variables $\mathbf { \Delta } _ { t _ { i } }$ and $t _ { j }$ , due to the unsquared norm term. To address this issue, the work [840] proposes an elegant reformulation of (6.32), which introduces auxiliary unit vectors $b _ { i j }$ (with $\| \pmb { b } _ { i j } \| = 1 )$ :

$$
\begin{array}{c}\min_{\substack{\boldsymbol{R}_{i}\in \mathrm{SO}(d)\\ \boldsymbol{t}_{i}\in \mathbb{R}^{d}\\ \boldsymbol{b}_{ij}\in S^{d - 1}}}\sum_{(i,j)\in \overrightarrow{\mathcal{E}}}\kappa_{ij}\| \boldsymbol{R}_{j} - \boldsymbol{R}_{i}\tilde{\boldsymbol{R}}_{ij}\|_{F}^{2} + \tau_{ij}\left\| \boldsymbol{t}_{j} - \boldsymbol{t}_{i} - \boldsymbol{R}_{i}\tilde{\boldsymbol{t}}_{ij}\right\|_{2}^{2}\\ \\ +\sum_{(i,j)\in \overrightarrow{\mathcal{E}}_{d}}\gamma_{ij}\left\| \boldsymbol{t}_{j} - \boldsymbol{t}_{i} - \tilde{r}_{ij}\boldsymbol{b}_{ij}\right\|_{2}^{2}, \end{array}\tag{6.33}
$$

Intuitively, formulation (6.33) also infers the bearing $b _ { i j }$ between variables i and $j$ for which a range measurement $\tilde { r } _ { i j }$ is available, and then recasts the corresponding term in the objective function, namely $\| \pmb { t } _ { j } - \pmb { t } _ { i } - \tilde { r } _ { i j } \pmb { b } _ { i j } \| ^ { 2 }$ , as a range-and-bearing measurement (similar to the ones we have seen in the previous sections). The advantage in doing so is that (6.33) is now a QCQP, since the last term in the objective is now quadratic in the unknowns, and the additional unit-norm constraints $\| \pmb { b } _ { i j } \| ^ { 2 } = 1$ on the $b _ { i j }$ are quadratic as well. The new formulation can then be relaxed to an SDP using Shor’s relaxation and solved eficiently using the Riemannian Staircase, leading to CORA [840], a fast certifiable algorithm for range-aided SLAM.

In contrast to the pose-and-landmark SLAM problems considered previously, the Shor relaxation of the range-aided SLAM problem (6.33) is not generically exact under bounded measurement noise. Specifically, the work [840] showed (empirically) that this relaxation is not typically exact in the multi-robot case unless there are relative pose measurements between the robots (i.e., range-only measurements between the robots do not sufice to obtain exact relations). However, the relaxation is still exact in a variety of practical SLAM problems, and allows computing certifiably optimal solutions with runtime comparable to local solvers. More generally, Papalia et al. [840] show that the connectivity of the graph underlying the SLAM problem largely impacts the exactness of the SDP relaxation, a phenomenon also observed in [937, 465]. As we will see in Section 6.2, the graph connectivity also afects the accuracy of the SLAM estimate, creating very interesting graph-theoretic insights into the SLAM problem.

## 6.1.4.2 Certifiable Algorithms Beyond Shor’s Relaxation: Anisotropic Noise and Outliers

So far we have reviewed certifiable algorithms (and fast solvers) for SLAM problems that can be reformulated as QCQPs. Below, we consider a broader class of SLAM problems, namely, problems with anisotropic measurement noise and problems with outliers. The interesting observation behind these problems is that, while strictly speaking they are no longer stated as QCQPs, they can often be formulated as Polynomial Optimization Problems (POPs), where both the objective and constraints are polynomial functions (instead of quadratic functions). This observation is important since there exists a generalization of Shor’s relaxation, namely the Moment (or Lasserre’s) Relaxation, that allows deriving SDP relaxations for POPs, thus enabling the design of certifiable algorithms for a broader set of problems. Below, we review examples of SLAM problems that can be written as POPs, and then provide an overview of the Moment Relaxation and practical considerations.

Example: Landmark-based SLAM with Anisotropic Noise. In previous sections, we assumed the measurement noise to be isotropic. However, measurements produced by common sensing modalities (e.g., stereo cameras, LiDAR, and radar) typically exhibit anisotropic noise. For instance, the measurement of the position of a landmark as observed by a stereo camera is typically more uncertain along the viewing direction of the cameras, due to the uncertainty induced by the stereo matching and triangulation process. More formally, the measurement model for the landmark measurements becomes $\tilde { m } _ { i k } = R _ { i } ^ { \top } ( m _ { k } - t _ { i } ) + \epsilon _ { i k }$ , where $\epsilon _ { i k } \sim \mathcal { N } ( \mathbf { 0 } , \mathbf { W } _ { i k } )$ and $\mathbf { W } _ { i k }$ is an anisotropic covariance $( i . e . , \mathbf { W } _ { i k }$ cannot be written as a scalar multiple of the identity matrix). In the presence of anisotropic noise, we need to generalize the landmark-based SLAM Problem 6.5 as follows [465]:

$$
\begin{array}{rl} & {\text{Problem 6.7 (Landmark - based SLAM with Anisotropic Noise)}}\\ & {\quad \underset { \begin{array}{c}\boldsymbol{R}_i\in \mathrm{SO}(d)\\ \boldsymbol{t}_i\in \mathbb{R}^d, \boldsymbol{m}_k\in \mathbb{R}^d \end{array}}{\min}\sum_{(i,j)\in \overrightarrow{\mathcal{E}}}\kappa_{ij}\| \boldsymbol{R}_j - \boldsymbol{R}_i\tilde{\boldsymbol{R}}_{ij}\| _F^2 +\tau_{ij}\left\| \boldsymbol {t}_j - \boldsymbol {t}_i - \boldsymbol{R}_i\tilde{\boldsymbol{t}}_{ij}\right\| _2^2}\\ & {\qquad +\sum_{(i,k)\in \vec{\mathcal{E}}_m}\| \boldsymbol{R}_i^\top (\boldsymbol {m}_k - \boldsymbol {t}_i) - \tilde{\boldsymbol{m}}_{ik}\|_{\mathbf{W}_{ik}}^2} \end{array}\tag{6.34}
$$

In (6.34), $\vec { \pmb { \varepsilon } } _ { m }$ denotes the set of edges corresponding to landmark measurements, and, for a generic vector a and matrix W of suitable dimensions, the notation $\| \pmb { a } \| _ { W } ^ { 2 } = \pmb { a } ^ { \top } W \pmb { a }$ denotes the standard Mahalanobis squared norm. This seemingly innocuous change with respect to Problem 6.5 is quite consequential in practice. Indeed, if $\mathbf { W } _ { i k }$ is isotropic $\left( e . g . , \mathbf { W } _ { i k } = \mu _ { i k } ^ { - 1 } \mathbf { I } _ { 3 } \right)$ we can manipulate the expression in (6.34) to the QCQP in Problem 6.5, whereas if $\mathbf { W } _ { i k }$ is anisotropic the problem is quartic $( i . e . ,$ it involves degree 4 polynomials in the variables).<sup>6</sup>

Example: SLAM with Outliers. So far, we assumed all measurements to be afected by zero-mean (but possibly anisotropic) Gaussian noise. Unfortunately, as we discussed in Chapter 3, in real SLAM problem some measurements might be outliers. This is typically the case for loop closure or landmark measurements, where incorrect place recognition or data association might cause adding incorrect measurements to the SLAM back-end. As we discussed in Chapter $^ { 3 , }$ an efective approach to mitigate the impact of outliers is to use robust loss functions. For instance, in the presence of outliers, the landmark-based SLAM Problem 6.5 becomes:

Problem 6.8 (Pose-Graph Optimization with Outliers)

$$
\begin{array}{r l} & {\underset { \begin{array}{c} \boldsymbol {R} _ {i} \in \mathrm{SO} (d) \\ \boldsymbol {t} _ {i} \in \mathbb {R} ^ {d} \end{array} } {\min} \sum_ {(i, j) \in \overrightarrow {\mathcal {E}} _ {o}} \kappa_ {i j} \| \boldsymbol {R} _ {j} - \boldsymbol {R} _ {i} \tilde {\boldsymbol {R}} _ {i j} \| _ {F} ^ {2} + \tau_ {i j} \left\| \boldsymbol {t} _ {j} - \boldsymbol {t} _ {i} - \boldsymbol {R} _ {i} \tilde {\boldsymbol {t}} _ {i j} \right\| _ {2} ^ {2}} \\ & {\quad + \sum_ {(i, k) \in \overrightarrow {\mathcal {E}} _ {m}} \rho \left(\sqrt {\mu_ {i k}} \left\| \boldsymbol {m} _ {k} - \boldsymbol {t} _ {i} - \boldsymbol {R} _ {i} \tilde {\boldsymbol {m}} _ {i k} \right\| _ {2}\right).} \end{array}\tag{6.35}
$$

In (6.35), $\rho ( \cdot )$ is a robust loss function (cf. Chapter 3) which is designed to reduce the influence of potential outliers in the landmark measurements. From Chapter $^ { 3 , }$ we know that we can use the Black-Rangarajan duality to reformulate (6.35) as a least-squares problem with auxiliary variables $w _ { i k } ,$ , one for each robust loss term. For instance, when choosing $\rho ( \cdot )$ to be the truncated quadratic loss, then (6.35) can

be rewritten as

$$
\begin{array}{l}\underset { \begin{array}{c}\boldsymbol{R}_{i}\in \mathrm{SO}(d)\\ \boldsymbol{t}_{i}\in \mathbb{R}^{d}\\ w_{ik}\in [0,1 ] \end{array} }{\min}\sum_{(i,j)\in \vec{\mathcal{E}}_{o}}\kappa_{ij}\| \boldsymbol{R}_{j} - \boldsymbol{R}_{i}\tilde{\boldsymbol{R}}_{ij}\|_{F}^{2} + \tau_{ij}\left\| \boldsymbol{t}_{j} - \boldsymbol{t}_{i} - \boldsymbol{R}_{i}\tilde{\boldsymbol{t}}_{ij}\right\|_{2}^{2}\\ \\ +\sum_{(i,k)\in \vec{\mathcal{E}}_{m}}w_{ik}\mu_{ik}\left\| \boldsymbol{m}_{k} - \boldsymbol{t}_{i} - \boldsymbol{R}_{i}\tilde{\boldsymbol{m}}_{ik}\right\|_{2}^{2} + (1 - w_{ik})\beta^{2}, \end{array}\tag{6.36}
$$

where $\beta$ is the maximum inlier error, as specified by the truncated quadratic loss. We note that (6.36) now includes auxiliary variables $w _ { i k }$ , which indicate whether a measurement is classified as an inlier or outlier. Interestingly, the objective function in (6.36) includes polynomials of degree up to 3, while the constraints are still at most quadratic functions. The work [1221] shows that the same conclusion holds for several choices of robust losses and for other variations of the problem, including pose-graph optimization and multiple rotation averaging. Below, we discussion how to obtain semidefinite relaxations for optimization problems involving polynomials.

Polynomial Optimization Problems and Moment Relaxation. The previous examples show how a broad range of SLAM problems can be reformulated as optimization problems involving polynomials. More formally, they can be written as POPs:

$$
\begin{array}{c} \min _ {\boldsymbol {x}} p (\boldsymbol {x}) \\ \text {subject to} h _ {i} (\boldsymbol {x}) = 0, i = 1, \ldots , n _ {h} \\ g _ {i} (\boldsymbol {x}) \leq 0, i = 1, \ldots , n _ {g}, \end{array}\tag{POP}
$$

where the functions $p , h _ { i } , g _ { i }$ are real polynomials in the variable ${ \mathbf { } } ^ { \mathbf { } } \mathbf { { \mathbf { x } } } ,$ and $n _ { h }$ and $n _ { g }$ are the number of equality and inequality constraints.

Rewriting the SLAM problem as (POP) does not immediately imply any computational advantage: POPs are a very general class of optimization problems (which also includes QCQPs) and are generally intractable to solve. Our interest towards (POP) stems from the fact that there exists a standard procedure, known as the moment (or Lasserre $\mathbf { \eta } _ { s } ^ { } )$ relaxation, to obtain a semidefinite relaxation of (POP). Even more interestingly, the procedure provides tools to obtain a hierarchy of relaxations and also guarantees that —under mild assumptions— certain relaxations in this hierarchy are exact. While we refer the reader to [147] and the seminal works [627, 626] for a more extensive introduction to the moment relaxation, below we provide a simple example to convey the underlying ideas.

To illustrate how to obtain a relaxation of a POP, consider the following problem:

$$
\begin{array}{c} \min _ {\boldsymbol {x}} p (\boldsymbol {x}) \\ \text { subject   to } h _ {i} (\boldsymbol {x}) = 0, i = 1, \ldots , n _ {h} \end{array}\tag{6.37}
$$

where, for simplicity, we assume $\pmb { x } = [ x _ { 1 } ; x _ { 2 } ]$ , and that $p ( { \pmb x } )$ and $h ( { \pmb x } )$ are polynomials of degree at most 4; in this simplified example we only have equality constraints.

Then, to derive the moment relaxation of (POP), we define the vector of monomials of degree up to $r = 2 .$ , where r is called the order of the relaxation:

$$
[ \pmb {x} ] _ {2} = [ 1; x _ {1}; x _ {2}; x _ {1} ^ {2}; x _ {1} x _ {2}; x _ {2} ^ {2} ].\tag{6.38}
$$

Then, we can form the moment matrix as the following outer product:

$$
\boldsymbol {X} _ {4} \triangleq [ \boldsymbol {x} ] _ {2} [ \boldsymbol {x} ] _ {2} ^ {\mathsf {T}} = \left[ \begin{array}{c c c c c c} 1 & x _ {1} & x _ {2} & x _ {1} ^ {2} & x _ {1} x _ {2} & x _ {2} ^ {2} \\ x _ {1} & x _ {1} ^ {2} & x _ {1} x _ {2} & x _ {1} ^ {3} & x _ {1} ^ {2} x _ {2} & x _ {1} x _ {2} ^ {2} \\ x _ {2} & x _ {1} x _ {2} & x _ {2} ^ {2} & x _ {1} ^ {2} x _ {2} & x _ {1} x _ {2} ^ {2} & x _ {2} ^ {3} \\ x _ {1} ^ {2} & x _ {1} ^ {3} & x _ {1} ^ {2} x _ {2} & x _ {1} ^ {4} & x _ {1} ^ {3} x _ {2} & x _ {1} ^ {2} x _ {2} ^ {2} \\ x _ {1} x _ {2} & x _ {1} ^ {2} x _ {2} & x _ {1} x _ {2} ^ {2} & x _ {1} ^ {3} x _ {2} & x _ {1} ^ {2} x _ {2} ^ {2} & x _ {1} x _ {2} ^ {3} \\ x _ {2} ^ {2} & x _ {1} x _ {2} ^ {2} & x _ {2} ^ {3} & x _ {1} ^ {2} x _ {2} ^ {2} & x _ {1} x _ {2} ^ {3} & x _ {2} ^ {4} \end{array} \right].\tag{6.39}
$$

Now the key observation is that we can write any polynomial of degree up to 4 as a linear combination of the entries of $X _ { 4 }$ . Therefore, we can rewrite (6.37) as:

$$
\begin{array}{c} \underset {\boldsymbol {X}, \boldsymbol {x}} {\min} \operatorname{tr} (\boldsymbol {C X}) \\ \text {subject to tr} (\boldsymbol {H} _ {i} \boldsymbol {X}) = 0, i = 1, \ldots , n _ {h} \\ \boldsymbol {X} = [ \boldsymbol {x} ] _ {2} [ \boldsymbol {x} ] _ {2} ^ {\mathsf {T}} \end{array}\tag{6.40}
$$

In full analogy with Shor’s relaxation, we can now replace the constraint $\boldsymbol { X } \ : =$ $[ { \pmb x } ] _ { 2 } [ { \pmb x } ] _ { 2 } ^ { \mathsf { T } }$ with $X \succeq 0$ and rank $( \pmb { X } ) = 1$ and then relax the rank constraint to obtain a semidefinite relaxation. Importantly, the moment relaxation also adds redundant $_ { c o n s t r a i n t s } 7$ to improve its quality. These constraints capture the fact that the moment matrix contains repeated entries $( e . g .$ , the term x<sub>1</sub>x<sub>2</sub> appears 3 times in (6.40)) as well as the fact that if $h _ { i } ( { \pmb x } ) = 0$ , then also $x _ { 1 } \cdot h _ { i } ( x ) = 0$ and $x _ { 2 } \cdot h _ { i } ( x ) = 0$ must hold. The moment relaxation provides a systematic way to identify all these redundant constraints, and also extends to the case of inequality constraints. Moreover, while above we derived an order $r = 2$ relaxation, we can repeat the procedure for any integer $r \geq 2 .$ , obtaining larger but better SDP relaxations (hence a hierarchy of relaxations). Indeed, the seminal work [806] establishes that the moment relaxation will produce exact relaxations $( i . e .$ , recovers a certifiably optimal solution to the original POP) at a finite order r, under mild assumptions. Interestingly, related work [1221, 465] has observed that moment relaxations of problems with anisotropic noise and outliers are already exact at a low relaxation order. Moreover, they remain tight even when using a subset of the variables in the monomial basis (6.38) (i.e., a sparse monomial basis), which further reduces the size of the resulting SDP.

The ‘Catch’: Solving the SDP Moment Relaxation. One important difference between the relaxations we have seen in the previous sections and the ones discussed in this section is that the latter involve a large number of redundant constraints in the SDP relaxation. While this diference might seem minor, the consequences for the SDP solver are profound. In particular, SDPs with redundant constraints (as the ones typically obtained from the moment relaxation) are degenerate [27], which makes the application of the Riemannian Staircase solver problematic for two reasons. First, constraint qualification conditions typically fail for these problems, and these conditions are required to ensure convergence of the Riemannian Staircase [841]. Second, degenerate SDPs have an infinite number of dual solutions, which creates computational obstacles in the implementation of the Riemannian Staircase. In other words, the Riemannian Staircase is no longer a viable solver for the degenerate SDPs typically produced by the moment relaxation. While the recent literature includes specialized solvers for moment relaxations of POPs [1220], these solvers are still relatively slow compared to local solvers.

## 6.2 How Accurate is the Optimal Solution of a SLAM Problem?

In the previous section we discussed how to obtain certifiably optimal solutions to certain SLAM problems. However, another fundamental question remains: how accurate is the optimal solution compared to the ground truth? Understanding this fundamental limit and identifying the key factors that influence estimation accuracy provides critical guidance for both system designers and end users. For example, at design time such insights can guide the choice of sensors the robot is equipped with; on the other hand, at deployment time, these insights can be used to guide the motion of the robot (and the corresponding acquisition of measurements) to ensure robust SLAM performance. Before ofering a concrete answer, we first need to formalize the question. Modern SLAM pipelines are complex, with many interacting subsystems afecting overall performance. Consequently, we approach this question from an estimation-theoretic perspective, focusing on the SLAM back-end.

Some Terminology and Facts. The goal of the SLAM back-end is to estimate unknown quantities such as robot poses and landmarks’ positions from noisy measurements. These measurements are random variables because they are corrupted by random sensor noise. An estimator is a function that maps collected noisy measurements to an estimate of the unknown parameters. Since the estimator depends on random measurements, it is itself a random variable. The Mean Squared Error (MSE) is a commonly used metric for evaluating an estimator’s performance. As the name suggests, the MSE represents the squared estimation error averaged over all possible measurements. In the univariate case, the MSE can be expressed as the sum of the estimator’s squared bias and its variance. The bias refers to the diference between the ground truth and the estimator’s output, averaged over all possible measurements. The variance captures the variability of the estimator’s output around its mean. The relationship between bias and variance, and the definition of the MSE extends naturally to the multivariate setting, where the variance is replaced by a covariance matrix. The MSE is still the sum of the squared norm of the bias and the trace of the covariance matrix. Clearly, for unbiased estimators (i.e., estimators with zero bias), the only quantity influencing the MSE is the covariance matrix. In the following, we show how to lower bound the covariance of the SLAM estimate, thus getting a fundamental limit on the accuracy achievable by a SLAM system.

## 6.2.1 Cram´er-Rao Lower Bound and the Fisher Information Matrix

The Cram´er-Rao Lower Bound (CRLB) provides a theoretical limit on the best estimator covariance achievable by any unbiased estimator. Formally,

$$
\mathrm{Cov} (\hat {\pmb {x}}) \succeq \mathcal {I} (\pmb {x} _ {\mathrm{true}}) ^ {- 1},\tag{6.41}
$$

where xˆ is any unbiased estimator of $\pmb { x } _ { \mathrm { t r u e } } \in \mathbb { R } ^ { m }$ , and $A \succeq B$ indicates that $\pmb { A } - \pmb { B }$ is positive semidefinite. The matrix $\mathcal { T } ( \pmb { x } _ { \mathrm { t r u e } } )$ appearing on the right-hand side of (6.41) is the Fisher information matrix (FIM), which is defined $\mathrm { a s } ^ { 8 }$

$$
[ \mathcal {I} (\pmb {x} _ {\mathrm{true}}) ] _ {i, j} \triangleq \mathbb {E} _ {\pmb {z}} \left[ \frac {\partial}{\partial x _ {i}} \log p (\pmb {z}; \pmb {x}) \frac {\partial}{\partial x _ {j}} \log p (\pmb {z}; \pmb {x}) \right].\tag{6.42}
$$

Here the expectation is taken over the possible realizations of measurements z drawn from the probability density function $p ( z ; x _ { \mathrm { t r u e } } )$ , and the partial derivatives of the log-likelihood function are evaluated at the true parameter value ${ \mathbf { \mathcal { x } } } _ { \mathrm { t r u e } }$ . Under certain regularity conditions, the FIM can also be expressed as the expected value of the Hessian of the log-likelihood:

$$
\left[ \mathcal {I} (\pmb {x} _ {\mathrm{true}}) \right] _ {i, j} = - \mathbb {E} _ {\pmb {z}} \left[ \frac {\partial^ {2}}{\partial x _ {i} \partial x _ {j}} \log p (\pmb {z}; \pmb {x}) \right].\tag{6.43}
$$

This connection to the Hessian provides an intuitive understanding of the CRLB: the CRLB establishes a lower bound on the covariance of any unbiased estimator, expressed in terms of the local sensitivity (curvature) of the (expected) loglikelihood function with respect to the parameters around the true parameter value ${ \bf { \mathit { x } } } _ { \mathrm { { t r u e } } }$ . If the log-likelihood is relatively flat around the true parameter value across diferent realizations of measurements $( i . e .$ , in expectation), any unbiased estimator will struggle to accurately localize the true parameter based on the observed data. In such cases, unbiased estimators will exhibit higher variance, leading to a higher MSE. In summary, the FIM captures the amount of information one can learn about the true parameter’s value from measurements using any unbiased estimator.

Under certain regularity conditions, it has been shown that the maximum likelihood estimator $\hat { \pmb x } _ { \mathrm { m l e } }$ asymptotically $( i . e .$ , when the number of measurements tends to ) converges (in distribution) to $\mathcal { N } \left( \pmb { x } _ { \mathrm { t r u e } } , \mathcal { T } ( \pmb { x } _ { \mathrm { t r u e } } ) ^ { - 1 } \right)$ . Therefore, the maximum likelihood estimator is asymptotically unbiased and achieves the CRLB $( i . e . ,$ , minimum variance among all unbiased estimators). Since the true value of parameters ${ \bf { \sigma } } _ { \bf { { x } \mathrm { { r u e } } } }$ is unknown, we often approximate the FIM by $\mathcal { T } ( \hat { \pmb x } _ { \mathrm { m l e } } )$ . Furthermore, it is common to approximate the covariance of the maximum likelihood estimator $\hat { \pmb x } _ { \mathrm { m l e } }$ with $\mathcal { T } ( \hat { x } _ { \mathrm { m l e } } ) ^ { - 1 }$

As an example, consider the common scenario where measurements are generated by corrupting a smooth (potentially nonlinear) function with additive Gaussian noise. In this case, the measurement model can be written as

$$
\pmb {z} = \pmb {h} (\pmb {x} _ {\mathrm{true}}) + \epsilon ,\tag{6.44}
$$

where $\epsilon \sim \mathcal { N } ( \mathbf { 0 } , \mathbf { \boldsymbol { \Sigma } } )$ is the noise. The likelihood function evaluated at x is thus given by $p ( z ; x ) = \mathcal { N } ( h ( x ) , \Sigma )$ . Plugging this likelihood in (6.42) yields the FIM (evaluated at x):

$$
\mathcal {I} (\pmb {x}) = \mathbf {J} (\pmb {x}) ^ {\top} \pmb {\Sigma} ^ {- 1} \mathbf {J} (\pmb {x}),\tag{6.45}
$$

in which $\mathbf { J } ( { \pmb x } )$ denotes the Jacobian of the measurement model h evaluated at x.<sup>9</sup> A Scalar Measure of Uncertainty: Optimal Experimental Design Criteria. In most applications, we would like our estimates to be as certain and accurate as possible. The FIM quantifies certainty, but it is generally matrix-valued; ideally we want a single value that quantifies uncertainty that we can use to make decisions or evaluate our estimator. Therefore, in many applications $( e . g . ,$ , system design, active SLAM), one needs to map the FIM, a PSD matrix, to a real number that captures a meaningful and ‘optimizable’ aspect of the estimation error uncertainty. Standard choices have been investigated in the field of Optimal Experimental Design. These include the determinant of the FIM (D-optimality), the trace of its inverse (A-optimality), and its smallest eigenvalue (E-optimality). These criteria are spectral functions of the FIM $( i . e . _ { \cdot }$ , can be computed from the eigenvalues of the FIM). Each criterion reflects a diferent aspect of estimation error: the D-optimality criterion quantifies the uncertainty hyper-volume, the A-optimality criterion measures the average variance, and the E-optimality criterion represents the worst-case estimation variance.

## 6.2.2 Fisher Information Matrix and Graph Laplacian

In this section, we study how the graphical structure of SLAM problems afects the FIM. Inspecting the FIM (6.45) reveals the intuitive fact that the measurement noise covariance matrix Σ afects the FIM: unsurprisingly, higher measurement noise covariance increases the lower bound on the smallest achievable MSE among all unbiased estimators in the CRLB. Additionally, the Jacobians of the measurements appear in (6.45), but it is dificult to intuitively understand from this equation how the measurements’ Jacobian afects the CRLB.<sup>10</sup> In the following, we gain insights on the structure of the Jacobians and the resulting FIM by relating them to properties of the graph underlying the SLAM problem.

All variants of SLAM naturally admit a graphical representation as we have seen in Chapter 1 and earlier in this chapter. The graph essentially encodes “who is observing what” and provides a concise overview of the SLAM problem. For instance, in pose-graph optimization and landmark-based SLAM, each variable (e.g., robot pose or landmark position) is represented by a vertex, while pairwise measurements (pose-pose or pose-landmark) correspond to edges between the respective vertices.

Next, we study how properties of the graph underlying the SLAM problem impact the accuracy of the resulting estimate. In particular, the degree of connectivity within the graph reflects the redundancy in measurements. Intuitively, a “better” connected SLAM graph is expected to be more robust to noise, yielding accurate estimates even under higher noise levels due to redundant measurements. While it is straightforward to show from (6.45) that introducing additional measurements (i.e., edges) always reduces the lower bound in the CRLB (in the Loewner order),<sup>11</sup> the impact of diferent measurements varies depending on which variables are involved in the additional measurements (i.e., the resulting graph connectivity). This is particularly evident in the context of loop closure in SLAM: closing a “larger” loop has a more significant efect on improving the accuracy of the SLAM solution. This intuition has been formalized in a series of works, establishing connections between the graphical structure of SLAM and desirable properties in both estimation and optimization. Below we provide a brief overview of these findings.

Connections between FIM and Graph Laplacian for a Simplified PGO Problem. The FIM in landmark-based SLAM and pose-graph optimization is closely connected to the graph Laplacian [561, 560, 877, 190]. This relationship is intuitive given that the measurements in these frameworks consist of pairwise relative observations between vertices. To illustrate the concept, we derive the FIM for the simpler problem of estimating robot positions in a 3D pose-graph optimization problem when the robot orientations are known. Let $z _ { k }$ denote the k-th relative measurement in which pose $i _ { k }$ observes pose $j _ { k }$ in its local frame:

$$
\boldsymbol {z} _ {k} = \boldsymbol {R} _ {i _ {k}} ^ {\top} (\boldsymbol {t} _ {j _ {k}} - \boldsymbol {t} _ {i _ {k}}) + \boldsymbol {\epsilon} _ {k},\tag{6.46}
$$

where $\boldsymbol \epsilon _ { k } \sim \mathcal N ( \mathbf 0 , w _ { k } ^ { - 1 } \mathbf I _ { 3 } )$ in which $\mathbf { I } _ { 3 }$ is the 3 3 identity matrix. Let z, t, and ϵ be the stacked vectors of measurements, positions, and noise variables. Additionally, let R be the block-diagonal matrix of (known) rotation matrices such that the k-th block is the rotation matrix involved in the k-th measurement:

$$
\boldsymbol {R} \triangleq \operatorname{BlockDiag} \left(\boldsymbol {R} _ {i _ {1}}, \boldsymbol {R} _ {i _ {2}}, \dots , \boldsymbol {R} _ {i _ {m}}\right).\tag{6.47}
$$

The stacked measurement model can be expressed as:

$$
\pmb {z} = \pmb {R} ^ {\top} (\pmb {A} \otimes \mathbf {I} _ {3}) ^ {\top} \pmb {t} + \pmb {\epsilon},\tag{6.48}
$$

where A denotes the reduced incidence matrix of the pose graph,<sup>12</sup> and $\otimes$ denotes the Kronecker product. Therefore, the Jacobian matrix of the stacked measurement model is given by

$$
\boldsymbol {J} = \boldsymbol {R} ^ {\top} (\boldsymbol {A} \otimes \mathbf {I} _ {3}) ^ {\top}.\tag{6.49}
$$

The information matrix of the stacked noise vector is given by

$$
\boldsymbol {\Sigma} ^ {- 1} = \operatorname{BlockDiag} \left(w _ {1} \mathbf {I} _ {3}, w _ {2} \mathbf {I} _ {3}, \dots , w _ {m} \mathbf {I} _ {3}\right).\tag{6.50}
$$

Let W denote the diagonal matrix of edge weights:

$$
\boldsymbol {W} \triangleq \left[ \begin{array}{c c c c} w _ {1} & 0 & \dots & 0 \\ 0 & w _ {2} & \dots & 0 \\ \vdots & \vdots & \ddots & \vdots \\ 0 & 0 & \dots & w _ {m} \end{array} \right].\tag{6.51}
$$

Using (6.45), the FIM can be computed as follows:

$$
\mathcal {I} = \boldsymbol {J} ^ {\top} \boldsymbol {\Sigma} ^ {- 1} \boldsymbol {J}\tag{6.52a}
$$

$$
= \left(\boldsymbol {A} \otimes \mathbf {I} _ {3}\right) \boldsymbol {R} \boldsymbol {\Sigma} ^ {- 1} \boldsymbol {R} ^ {\top} \left(\boldsymbol {A} \otimes \mathbf {I} _ {3}\right) ^ {\top}\tag{6.52b}
$$

$$
= (\boldsymbol {A} \otimes \mathbf {I} _ {3}) (\boldsymbol {W} \otimes \mathbf {I} _ {3}) (\boldsymbol {A} \otimes \mathbf {I} _ {3}) ^ {\top}\tag{6.52c}
$$

$$
= (\boldsymbol {A} \boldsymbol {W} \boldsymbol {A} ^ {\top}) \otimes \mathbf {I} _ {3}\tag{6.52d}
$$

$$
= \pmb {L} _ {w} \otimes \mathbf {I} _ {3},\tag{6.52e}
$$

where we observed that the block diagonal entries of $\Sigma ^ { - 1 }$ are invariant to rotation, hence $R \Sigma ^ { - 1 } R ^ { \top } = \Sigma ^ { - 1 }$ , and we used the fact that $\Sigma ^ { - 1 } = { \pmb { W } } \otimes { \bf { I } } _ { 3 }$ . In (6.52a), $\mathbf { } L _ { w } = A W A ^ { \top }$ is the reduced weighted Laplacian matrix of the graph where edge weights are given by $w _ { 1 } , w _ { 2 } , \ldots , w _ { m }$ . This clearly shows that in this case (simplified PGO where robot orientations are assumed known and noise is assumed isotropic), the FIM is fully characterized by the reduced weighted Laplacian of the underlying graph.

FIM for Other SLAM Problems. The result above can be generalized to PGO and landmark-based SLAM problems [561, 877, 190]. In these problems, the FIM involves both the reduced weighted Laplacian matrix of graph and additional terms that depend on the robot trajectory. In particular, the FIM for 3D pose-graph optimization can be written as [877, Eq. 29]:

$$
\mathcal {I} (\mathbf {x}) = \sum_ {k = 1} ^ {m} \boldsymbol {L} _ {k} \otimes \left(\operatorname{Ad} \left(\boldsymbol {T} _ {i _ {k}} ^ {- 1}\right) ^ {\top} \boldsymbol {\Sigma} _ {k} ^ {- 1} \operatorname{Ad} \left(\boldsymbol {T} _ {i _ {k}} ^ {- 1}\right)\right),\tag{6.53}
$$

where $\pmb { T } _ { i _ { k } }$ is the pose of the robot making the k-th measurement, $\Sigma _ { k }$ is the covariance matrix of the noise corrupting the k-th measurement, and $\scriptstyle L _ { k }$ is the reduced elementary Laplacian matrix of the k-th edge defined as $\mathbf { } \mathbf { } L _ { k } \triangleq \mathbf { a } _ { k } \mathbf { a } _ { k } ^ { \intercal }$ where $\mathbf { \em a } _ { k }$ is the k-th column of the reduced incidence matrix of the graph. Empirical results and theoretical analyses demonstrate that, under certain conditions, the (approximate) optimal design criteria derived from the reduced Laplacian closely match those obtained from the FIM [561, 877, 190].

Practical Considerations. The connection between the FIM and the graph Laplacian allows approximating optimal design criteria —which are spectral functions of the FIM— using the spectrum of the reduced weighted Laplacian. For instance, the D-optimality criterion can be approximated by a function of the determinant of the weighted reduced Laplacian. According to Kirchhof’s matrix-tree theorem, this determinant equals the weighted number of spanning trees of the graph [561]. Since the weighted number of spanning trees serves as a measure of connectivity in edge-weighted graphs, this result formalizes the intuition that a graph’s connectivity directly influences estimation accuracy. Similarly, the Eoptimality criterion is related to the graph’s algebraic connectivity [275, 561, 877]. Overall, measuring uncertainty using the graph Laplacian instead of the FIM offers two main advantages: it leads to more computationally eficient techniques $( e . g .$ , the Laplacian has a dimension six times smaller than that of the FIM in 3D pose-graph optimization), and it eliminates the need to solve the SLAM problem or acquire actual measurements, as the calculations are based solely on the graph structure. These graphical approximations of design criteria have been successfully applied in active SLAM [190, 582, 877] —where a robot plans its trajectory to maximize the anticipated SLAM accuracy— and in measurement selection and pruning [275, 561, 1094], where the goal is to select and retain only the most informative measurements in lifelong SLAM problems.

## 6.3 Further Readings & Recent Trends

Certifiable Algorithms. The first certifiable algorithm for 2D SLAM traces back to [149], and builds on earlier work done in computer vision and related areas, including [338, 1014]. Extensions to 3D SLAM and variations quickly followed [151, 1108, 937, 939, 114, 152], with SE-Sync providing the first blueprint for building fast certifiable algorithms, as well as the first guarantees of global optimality $( i . e . ,$ exactness of the relaxation) under bounded measurement noise [937].

Since then, certifiable algorithms have been designed for a variety of problems related to SLAM,<sup>13</sup> including rotation averaging [122, 972, 251], PGO [937, 149], landmark-based SLAM [465], multi-robot SLAM [1093], range-aided SLAM [840], 3D registration [115, 1223], multi-set registration [501], 2-view geometry [116, 1285, 367, 1097, 538], perspective-n-point problems [1053], calibration [382], single-frame pose and shape estimation [1006], multi-frame pose and shape estimation [994], and structure from motion with learned depth [1256]. Recent work has also extended certifiable algorithms to cope with anisotropic noise [466] and outliers [1221].

While there has been incredible progress in this area over the last decade, three exciting open questions remain. First of all, there are still SLAM problems that cannot be attacked with certifiable algorithms. For instance, when doing visual SLAM (Chapter 7), the perspective projection arising in the objective function is a rational function rather than a polynomial; while the problem can still be reformulated as a POP by introducing extra variables, it is typically impractical to add one variable for each keypoint measurement in an image. Similarly, the modeling of IMU measurements (that we discuss in detail in Chapter 11) has not been conducive to the design of certifiable algorithms. Second, except for certain problems, e.g., [937, 538, 1256, 840], where the Riemannian Staircase method can be applied as a fast solver, SDPs might need to be solved by interior-point methods or other ad hoc solvers. While interior-point methods are very efective and fast in solving low-dimensional optimization problems arising in the SLAM front-end (e.g., [115, 1223, 116, 1285, 367, 1097, 538]), they become impractically slow when applied to large-scale problems in the SLAM back-end (e.g., [937]), and ad hoc solvers —while being more scalable— are still relatively slow compared to local solvers [1220]. Recent work to address this problem has not only investigated faster SDP solvers, but has also focused on how to reduce the size of the SDPs (e.g., by sparsifying the monomial basis underlying the moment relaxation) [1220], or how to reduce the number of constraints, in the attempt to make the SDPs non-degenerate or faster to solve [290]. Finally, while current works compute per-instance certificates of optimality (i.e., they compute an estimate and possibly provide a certificate of optimality for it), the literature lacks a fundamental understanding of when the relaxation is expected to be exact, with only few papers providing conditions for exactness in specific problems [1218, 859, 937, 310, 147].

Problems with Outliers. Real-world SLAM problems are typically plagued with outliers, a problem that we discussed at length in Chapter 3. While several works wrap outlier-free certifiable algorithms in a Graduated Non-Convexity outer loop [1223, 1006, 994] to gain empirical robustness to outliers, recent work directly attempts the develop certifiable algorithms for robust estimation problems involving outliers. Eforts in this direction tackled PGO [618, 153], rotation estimation [1218], 3D registration [1223], multiple rotation averaging, absolute pose estimation, and pose and shape estimation [1221]. A good summary of these results is provided in [1221], while connections with parallel work in robust statistics are discussed in [147]. These algorithms rely on the moment relaxation described earlier in this chapter and the resulting SDPs are still relatively slow to solve. An alternative approach has been to obtain solutions via local solvers, and use the same insights behind the moment relaxation to derive methods to only check optimality [1219].

Many interesting open questions also remain in this case.<sup>14</sup> In the presence of outliers, we still see the same challenges and opportunities discussed in the previous paragraph, including how to extend certifiable algorithms to other problems with outliers, how to design faster solvers, and how to derive conditions for exactness (e.g., as a function of the amount of noise and outliers). At the same time, there are additional challenges related to the presence of outliers. First of all, the SLAM approaches discussed above assume an outlier-free odometry backbone: in other words, only the loop closure measurements are wrapped in a robust loss function. The assumption of having a reliable odometry source is acceptable in many SLAM problems, but can be restrictive in certain cases. For instance, in visual SLAM problems, the odometry can become unreliable if the feature tracking fails, while in multi-robot SLAM problems, there is no odometry backbone connecting the diferent robots and all the inter-robot measurements can be outliers. While the formulation of current certifiable algorithms would extend to the case where also the odometry is wrapped in a robust loss, the resulting relaxations are known to be loose, e.g., [618], and it is unclear how to improve them. Second, even if we compute an optimal estimate when solving a robust estimation problem, if the majority of the measurements are outliers, the estimate can still be grossly incorrect. An interesting and relatively unexplored area is to design certifiable algorithms that can recover multiple hypotheses while guaranteeing that at least one hypothesis is correct, a setting called list decodable regression in statistics [147].

Uncertainty Quantification and Downstream Applications. While Section 6.2 provided computational tools to bound the uncertainty in our SLAM estimate and potentially predict its evolution as a function of the structure of the underlying graph, it still leaves many questions open. First of all, the computation of the covariance of the SLAM estimate relies upon the knowledge of the measurement covariances: if the measurement covariance matrices are inaccurate, the resulting uncertainty bounds become unreliable. Recent work uses learning to estimate the measurement covariances or even the entire measurement model [1020, 889, 1244] (this is also related to our discussion of diferentiable optimization in Chapter 4). Second, the traditional approach to compute a covariance for the estimate does not account for the potential presence of outliers. In the presence of outliers, the estimated covariance can be incorrect and the distribution of potential estimates (e.g., of the robot trajectory) can become highly multimodal, hence limiting the use of the covariance estimates described earlier in this chapter. Finally, there is a growing literature using uncertainty quantification to guide active perception and active SLAM (see [876] for a survey) and to determine how to subselect measurements during life-long SLAM or in the presence of resource constraints [275, 561, 1094, 150].

PART II SLAM IN PRACTICE
II

Prelude

Ayoung Kim, Timothy Barfoot, Luca Carlone, Frank Dellaert, and Daniel Cremers

Part I laid the foundations of SLAM by introducing the basic language of factor graphs and their role as a SLAM back-end. Building on this groundwork, Part II focuses on the characteristics and integration of various sensor modalities used in SLAM, which directly impacts the SLAM front-end. Together, these two parts provide a cohesive understanding of the SLAM pipeline: the back-end, discussed in Part I, addresses the underlying optimization problem, while the front-end, covered in this part, tackles sensor-specific tasks such as pre-processing, time synchronization, noise filtering, and measurement modeling.

This part explores widely adopted sensors in SLAM, and the corresponding chapters are organized —based on the common categorization of SLAM algorithms— by their primary sensor type. The sensors discussed in Part II include RGB cameras and LiDARs, along with emerging modalities such as event cameras and radars. IMUs are emphasized for their critical role in SLAM, both as standalone sensors and in combination with other modalities. The discussion also explores how robot kinematics can be integrated as measurements in the SLAM system.

## II.1 Key Modules in the SLAM Front-End

Most SLAM front-ends rely on two key modules: odometry and loop closure. While these modules produce essential measurements, priors can be occasionally incorporated alongside them. The odometry module produces measurement between consecutive nodes in a graph, typically arranged in sequential time order. By chaining together these odometry measurements, a trajectory can be inferred, though it will inevitably accumulate drift over time. The accumulated drift highlights the importance of the loop closure module as a key component in the SLAM system, as it mitigates this drift by recognizing previously visited scenes and establishing connections to historical nodes. Incorporating loop closure detection and correction is the essence of SLAM. Beyond these two key modules, additional sensor measurements may also be included. For instance, one can add unary factors modeling additional sensor data or priors (e.g., GPS or pose priors), enhancing the overall SLAM system’s accuracy and robustness.

## II.1.1 Odometry

The odometry module estimates the relative transformation between two consecutive nodes in a factor graph. This transformation can vary in complexity depending on the scenario. The most common case involves two nodes corresponding to consecutive sensor frames, where a relative 6-DOF binary factor is inferred between them. In this context, the comparison of sensor measurements is performed by computing pixel-to-pixel, point-to-point, or feature-to-feature matching. Many visual and LiDAR SLAM systems compute odometry in this manner.

At times, the odometry computation does not rely on matching consecutive measurements. When kinematic or dynamic information is available, such as from radar or leg encoders and contact sensors, the odometry can be directly computed by velocity integration. For instance, in radar, both range and radial velocity can be measured. By using the radial velocity to infer ego-velocity, a 6-DOF transformation can be integrated between two frames. Leg odometry can be obtained by incorporating data from encoders or contact sensors, along with robot kinematics. One of the most widely used odometry methods is inertial odometry. Inertial measurements involve more complex computations between two nodes, often implemented as a preintegration factor. This preintegration can serve as odometry but is frequently combined with other sensors to reduce the odometric drift.

The odometry estimation module is typically named after the primary sensor used for estimation, e.g., visual odometry, LiDAR odometry, visual-inertial odometry, leg odometry, and so on. In this part, each chapter provides a detailed exploration of the odometry module tailored to specific sensor modalities.

## II.1.2 Loop Closure

The loop closure module addresses two problems: loop closure detection and relative pose estimation. The first problem, which is also referred to as place recognition, involves using information-retrieval methods to identify a candidate loop closure in a topological manner. This task involves identifying a query’s nearest index from a database, using retrieval or matching algorithms. In SLAM, the query is the current sensor measurement and the database includes a collection of previously observed places in the form of raw sensor measurements or descriptors.<sup>1</sup> Then the goal is to find a place in the database that matches the current observation, hence detecting the case where the robot is revisiting a previously explored area.

Loop closure detection is commonly performed using a variety of exteroceptive sensors, such as RGB cameras, event cameras, LiDARs, and radars. Early work focused on creating highly descriptive and compact place (or scene) descriptors. For instance, visual place recognition applied information retrieval techniques, introducing visual words to implement a bag-of-words model. Primarily, binary bag-of-words (DBoW) [362] has been widely adopted in many visual SLAM applications. Similarly, for range sensors, compact descriptors for range measurements, such as those used in Scan Context [563], have been developed. These hand-crafted descriptors are now transitioning to learning-based approaches, including cross-modal place recognition, which enables place recognition from diferent sensor modalities.

When detecting a loop closure, both discernibility and scalability are crucial. Place recognition must accurately distinguish between similar-looking but distinct locations, mitigating perceptual aliasing. To achieve this, it is essential to develop efective descriptors or train networks that ensure strong discernibility. In addition, during long-term SLAM operations over large areas, both the map and the query database will grow. Consequently, loop closure detection must be performed eficiently, even as the map expands.

Once a candidate is found, re-localization or relative pose estimation aims to estimate a fine registration between the candidate match and the current data, which is needed for the SLAM back-end optimization. Once the proposal loop closure candidate is detected, the follow-up registration estimates a relative transformation (either full 6-DOF or partial) between the query node and the candidate match node, typically resulting in a binary factor.

## II.1.3 Priors and Unary Factors

Some sensors measure properties of a single node in the factor graph rather than providing relative measurements between nodes. This is the case for sensors like GPS in terrestrial navigation, depth sensors in underwater environments, or radar providing instantaneous velocity measurements. These measurements can often be incorporated into the SLAM system as unary factors, constraining a single node.

The factor graph might also include priors, for instance enforcing the first node to be at the origin of the world frame, or potentially enforcing desired motion properties (e.g., zero-velocity at certain nodes). Unary factors and priors will not be extensively discussed in the following chapters, and readers are referred to dedicated resources on GNSS [145] or UWB applications.

## II.2 Sensors and Factor Graphs

From the factor-graph SLAM perspective, each sensing modality produces a type of factor, acting as a building block for the entire factor graph. An example factor graph is given in Figure II.1. The factor-graph framework, introduced in Part I, serves as a generic optimization back-end, where each sensor can contribute to the graph either independently or by being combined with other sensors.

![](images/aa081d6a4af84052ad891b7d0a207618b05d1f794c475c29d5e31f713a72d4e9.jpg)  
Figure II.1 Sample factor-graph including all sensing modalities introduced in this part. Here the variable nodes $p _ { i }$ are generic states (e.g., poses, velocities, etc.) at each time i.

## II.2.1 Selecting the Right Sensor for Your Application

Let us briefly overview exteroceptive sensors commonly used in SLAM systems, highlighting their advantages and limitations.

RGB Camera: RGB cameras are among the most widely adopted sensors in SLAM applications due to their afordability, ability to capture semantic information and appearance, and their resemblance to the human visual system. These cameras capture rich visual context, including semantic information, which serves as a powerful cue for various visual SLAM tasks.

However, they do have notable limitations. One significant drawback is their reliance on adequate lighting conditions; without suficient illumination, the quality of captured images degrades, which can hinder performance in low-light environments. Additionally, RGB cameras are sensitive to glare, reflections, motion blur, and shadows, and their performance is significantly limited in extreme environments, such as those with fog or smoke.

LiDAR: LiDARs directly measure distance without the need for triangulation, enabling to accurately capture 2D/3D geometry from range measurements. Indeed, range sensors, such as 2D LiDAR and sonar, are foundational sensing modalities in the history of SLAM. The point cloud data generated by LiDAR has been widely adopted for dense 2D/3D mapping in both indoor and outdoor environments, making it especially valuable for large-scale applications in construction sites, urban areas, and forests.

However, LiDARs remain expensive in terms of cost, memory and power consumption, as well as downstream computational requirements. In particular, 3D LiDARs generate large point clouds at each scans, requiring substantial computational power and memory for processing and storage. Moreover, 3D LiDAR sensors are still bulky, and size and weight requirements become a consideration, as compared to smaller sensors, such as RGB cameras. These challenges should be carefully considered when using LiDAR in budget-sensitive, real-time systems (e.g., drones).

Furthermore, despite their robustness to illumination conditions, their performance can be impaired in adverse weather conditions, such as heavy rain, fog, or snow.

Radar: Radar is less afected by environmental conditions, making it a reliable sensor for deployment in extreme environments. It can provide velocity information through Doppler measurements, adding an additional layer of functionality. While the localization and mapping accuracy of radar is often lower than that of LiDAR, it remains one of the few robust sensors capable of enabling SLAM in extreme environments where LiDAR and cameras may be impaired. However, radar data tends to be noisy and extremely sparse, which can complicate its processing. The signal processing required for radar data is also computationally intensive, and the sensor measurements are not as intuitive as those from optical sensors, making them more challenging to interpret and integrate into systems.

Event camera: Event cameras are unique sensors that ofer extremely high temporal resolution (in the order of microseconds) at a low power consumption. By transmitting data only for pixels that change, they are highly energy-eficient, and capture fast-moving objects without motion blur. Their asynchronous nature also leads to eficient data processing, generating smaller data streams compared to conventional frame-based cameras that operate at the same sampling rate. These features make event cameras ideal for real-time, high-speed applications. Additionally, their ability to handle both very bright and dark conditions with high dynamic range makes them more efective than frame-based cameras in challenging lighting environments. Despite their advantages, their main limitations are their noise, and lack of fine texture details and absolute intensity output.

## II.2.2 Sensor Fusion

In the SLAM literature, it is common practice to combine multiple complementary sensors to obtain high-performing systems. Inertial measurement units (IMUs) are often used due to their pervasiveness and low-SWAP nature. Incorporating an IMU with other sensors significantly enhances the performance of many SLAM systems. IMUs provide valuable information about motion, velocity, and orientation, which improves the accuracy and robustness of systems like visual-inertial odometry, LiDAR-inertial odometry, and radar-inertial odometry. These systems leverage the preintegration of IMU data to maintain continuous tracking and reduce drift over time. While IMU-only systems have seen some development, they remain limited in terms of accuracy and reliability. When combined with other sensors like LiDAR or cameras, however, IMUs strengthen the overall SLAM system, compensating for the weaknesses of individual sensors and enabling more precise and reliable mapping and localization, especially in dynamic or challenging environments.

Incorporating the kinematics of the robot platform is also crucial. Many SLAM systems in robotics are designed to work with specific platforms, such as ground robots, drones, or legged robots. By formulating the kinematics of these platforms and integrating them with other sensor modalities, the performance and robustness of the system can be significantly enhanced.

As we mentioned above, sensor fusion can further benefit from the complementarity and heterogeneity of the sensors used in the SLAM system. Camera and LiDAR systems are one of the most popular combinations. This is because semantic information from camera and direct range measurement from LiDAR can efectively compensate for each sensor’s limitation. Camera-radar fusion has similar benefits, complementing each other to mutually enhance performance and address limitations. For instance, the velocity measurements from radar can be used to detect dynamic objects instantaneously, while the semantic information from the RGB camera adds valuable context. Similarly, the combination of LiDAR and radar is beneficial, as radar can improve odometry and facilitate dynamic object removal, while the dense point clouds from LiDAR contribute to higher-quality mapping and submap matching.

Lastly, there are often heterogeneous characteristics even among sensors of the same type. RGB cameras, for instance, can vary significantly depending on factors such as lens type and shutter mechanism. When combining multiple LiDARs, the beam pattern, point cloud density, and field of view (FOV) can vary significantly between diferent LiDAR models, and this discrepancy must be addressed when integrating LiDAR with other sensors. This is also true for radar, where the two main types —spinning radar and SoC radar— difer significantly in terms of data type, measurement techniques, and their applications.

## II.2.3 Calibration and Synchronization of Sensors

Sensor calibration can be categorized into two types: intrinsic and extrinsic. Intrinsic calibration focuses on determining the model parameters specific to a sensor, such as the focal length and distortion coeficients for a camera or the intensity calibration for LiDAR. In the case of a camera using a pinhole camera model, intrinsic calibration estimates parameters like focal length and distortion coeficients. Intrinsic calibration begins with a sensor model and solves for the parameters that define that model.

On the other hand, extrinsic calibration involves finding the transformation between multiple sensors, aligning their coordinate systems, and ensuring accurate data fusion. This process includes establishing correspondences and using them in an optimization problem to solve for the relative transformation between two sensors. The core challenge arises when establishing correspondences across diferent modalities. For example, to match a pixel from an RGB camera to a 3D point from a LiDAR, one needs to solve a multi-modal registration problem. Due to diferences in data formats and underlying physics, comparing data from two sensors is often challenging.

Both intrinsic and extrinsic are typically performed in the factory, before the actual robot deployment. A widely adopted solution is to use a target to carefully capture data from both sensors in order to solve the registration problem. However, this calibration is often limited to the target and may sufer from drift over time. To address this, targetless calibration methods have been developed, which leverage the surroundings as calibration features. Alternatively, calibration can be updated adaptively during deployment by including the calibration parameters in the SLAM factor-graph optimization formulation.

When integrating multiple sensors into a SLAM system, time synchronization must be carefully managed. Each sensor operates at its own sampling rate, meaning data from diferent sensors arrives at diferent intervals. In robotics, the movement of the platform further exacerbates this issue, as the motion induces discrepancies between sensor data streams. Proper synchronization ensures that all sensor data is aligned in time, enabling accurate fusion and minimizing errors in mapping and localization. Of course, strict hardware synchronization is not always feasible, and interpolation —potentially using faster sampling sensors like an IMU— may be necessary to bridge the gap between sensor data streams.

## II.3 Evaluation

SLAM evaluation is typically conducted from multiple perspectives, often at the level of individual modules such as odometry, place recognition, and mapping. As expected, real-time performance is crucial for odometry, whereas place recognition and mapping can tolerate slower processing rates. More computationally intensive tasks, such as map maintenance and updates, are often deferred to post-processing.

The most common SLAM evaluation metric is the trajectory accuracy, typically measured by comparing the estimated path to a ground-truth trajectory. However, generating ground truth requires expensive sensors and often extensive postprocessing. While this trajectory-to-trajectory comparison sounds straightforward, it is not always feasible —especially in environments like indoors, where RTK GPS is inefective. In such cases, a few surveyed points using QR markers or artificial targets can serve as an alternative. Furthermore, SLAM performance can be analyzed from multiple perspectives by focusing on diferent quantitative metrics [373, 1283, 405] during trajectory evaluation.

For place recognition, standard classification metrics such as the precision-recall curve, AUC, and F1 score are commonly used. In the context of robot re-localization, however, not only the accuracy metrics but also the distribution [568] of candidate matches plays a critical role in assessing performance.

Lastly, map evaluation is often the most challenging aspect of the SLAM evaluation, not only due to the large spatial scale but also because of the dificulty in obtaining accurate ground truth. Static LiDAR systems, such as terrestrial laser scanners (TLS), are commonly used to create high-fidelity reference maps for comparison. In addition to global accuracy, evaluations should also consider fine-grained structural details and the memory eficiency of the map representation.

## II.4 How to Read Part II?

The chapters in this part are organized by sensing modality. Each chapter follows a similar structure, focusing on odometry, place recognition, and overall SLAM system for each sensor. It is recommended to start with the visual SLAM chapter to grasp the basic definitions of each SLAM module.

Following the discussion on visual SLAM in Chapter 7, two range sensors are introduced: LiDAR SLAM in Chapter 8 and radar SLAM in Chapter 9. Moving beyond conventional sensors, we delve into event camera SLAM in Chapter 10. Inertial measurement units (IMUs) are covered in Chapter 11. Additionally, this part explores how to model odometry for legged-robot systems in Chapter 12.

7

Visual SLAM

Jakob Engel, Juan D. Tard´os, Javier Civera, Margarita Chli, Stefan Leutenegger, Frank Dellaert, and Daniel Cremers

Reconstructing the world and the sensor motion from cameras is a challenge referred to as Visual Simultaneous Localization and Mapping, shortened to Visual SLAM or VSLAM. With cameras being omni-present, inexpensive, and power-eficient, the potential of visual SLAM for autonomous robots, self-driving cars, or mixed and augmented reality is endless. In this chapter, we first provide some historical background and terminology (Section 7.1), and give an overview of a typical visual SLAM pipeline (Section 7.2). Then, we review key ingredients of visual SLAM formulations (Section 7.3), and cover more advanced topics in image alignment and bundle adjustment (BA) (Section 7.4). Afterwards, we describe examples of visual SLAM systems (Chapter 7.5), real-time 3D reconstruction (Chapter 7.6), SLAM with depth-cameras (Chapter 7.7), and discuss the advantages of combining vision with other sensing modalities, including IMU, GPS, and WiFi (Chapter 7.8). We close the chapter with a brief discussion about new trends (Chapter 7.9).

## 7.1 Historical Background and Terminology

## 7.1.1 From Photogrammetry to Bundle Adjustment and Visual SLAM

Visual SLAM’s history builds upon general SLAM methods, but it is also rooted in advances from the photogrammetry and computer vision communities. We highlight here the historical connection with these two last fields, the connection with SLAM being already addressed throughout the handbook.

In 1822, Nic´ephore Ni´epce invented modern photography with the oldest surviving photograph being “A View from a Window at Le Gras” in 1827. The history of reconstructing the world from cameras started a few decades after the invention of photography. The French army oficer Aim´e Laussedat is often considered the inventor of photogrammetry as he pioneered the use of terrestrial photographs for topographic mapping around 1851. In 1867 the German engineer Albrecht Meydenbauer further developed photogrammetry for architectural surveys. From 1890 onward the German mathematician and mountaineer Sebastian Finsterwalder (president of the German Mathematical Society from 1915 onward) pioneered the use of aerial imagery for photogrammetric reconstruction of glaciers in the Alps and also advocated techniques of projective geometry [357]. His doctoral student Otto von Gruber formalized the mathematical framework of bundle adjustment (BA) for reconstruction of structure and motion from a set of corresponding points observed in multiple images. These concepts were developed around the beginning of the 20th century, well before the advent of computers. The deployment on computers was later pioneered by Hellmut H. Schmid, a German rocket scientist who developed matrix computation techniques for BA and teamed up with American Duane C. Brown in the 1950s to deploy these methods on the largest computers of their time.

Reconstruction methods and the field of structure from motion (SFM) research build on various camera models starting from the pinhole camera model and the use of projective geometry to capture the relationship between 2D point observations and their corresponding 3D world coordinates. In the early 1990s Tomasi and Kanade [1102] developed matrix factorization techniques for the reconstruction of static scenes under the simplified assumption of an orthographic projection. Whereas earlier approaches often focused on the reconstruction from two views, in the 2000s the community shifted to the problem of multi-view structure from motion. Traditionally, the reconstruction pipeline involved feature extraction, correspondence estimation, the use of minimal solutions for obtaining an initial camera configuration and a subsequent BA to obtain a globally consistent reconstruction. Much efort was therefore dedicated to the development of feature extraction and matching algorithms with feature descriptors such as SIFT [699] or SURF [62], and more recently a multitude of learning-based descriptors. In order to cope with incorrect point correspondences researchers developed sampling strategies such as RANSAC [328] that allowed the method to revisit the correspondence estimation in alternation with model fitting (cf. Chapter 3).

Whereas structure from motion is often focused on the accurate (generally ofline) reconstruction of large-scale 3D worlds from an unordered collection of images, visual SLAM typically focuses on the online and real-time reconstruction from a moving camera. A prerequisite for such online and real-time approaches was therefore the development of causal methods such as [209] which focus on the challenge of optimal structure from motion given only the past images (as opposed to the entire image collection or video). The first real-time capable methods for structure from motion / visual SLAM emerged around 2000 [524, 244].

## 7.1.2 Terminology

The following terms are often used interchangeably to describe similar processes; however, they place diferent emphasis depending on application and community they are used in.

Photogrammetry is the science of extracting accurate measurements, spatial information, and 3D reconstructions from 2D photographs. By analyzing overlapping images taken from diferent viewpoints, photogrammetry enables the creation of geometric representations of objects or environments. This technique is widely used in mapping, surveying, and 3D modeling, forming the foundation for many visual odometry and SLAM algorithms.

Bundle adjustment (BA) is a mathematical optimization method used to refine 3D reconstructions. It adjusts the positions, orientations, and optionally intrinsic parameters of cameras, along with the 3D positions of observed points, to minimize the reprojection error —the diference between observed image points and the points projected from the 3D model. Optimization is typically performed using second-order methods such as Gauss-Newton or Levenberg-Marquardt and requires careful initialization and robust outlier rejection to converge efectively. Recent research has also explored initialization-free BA, which aims to simplify the optimization process.

Structure from motion (SFM) refers to the process of reconstructing 3D structures from a collection of 2D images taken from diferent perspectives. Unlike photogrammetry, which often assumes known camera positions, SFM simultaneously estimates both the 3D structure of the scene and the motion of the cameras. SFM is typically applied in non-causal, non-real-time scenarios, where images may come from diverse sources (e.g., internet photo collections) rather than a continuous video stream. Typically in the final stage SfM will resort to BA for optimization. Landmark projects, such as Building Rome in a Day [14], demonstrate its scalability and potential for large-scale applications.

Visual odometry (VO) focuses on estimating the motion of a camera by analyzing sequential visual frames in a video. It identifies visual correspondences between consecutive images to measure the relative motion of the sensor over time. VO primarily deals with local motion estimation and operates within a sliding window of recent observations, without building a global map. However, VO is often combined with a loop closure detection and mapping component to form a complete visual SLAM system.

Visual SLAM is a computational technique that enables a system to simultaneously localize itself within an unknown environment and build a map of that environment in real time. It combines elements of photogrammetry, visual odometry, and structure from motion to process image data, track camera motion, and construct detailed 3D maps. In contrast to VO, it will typically employ an explicit functionality of recognizing previously visited places, re-localize relative to them, and optionally adjust pose estimates around such a “loop” —a process referred to as loop closure. Visual SLAM is a cornerstone technology for robotics, autonomous vehicles, and augmented reality, where precise navigation and environmental understanding are essential.

## 7.2 The Processing Pipeline of a Visual SLAM System

Building a complete visual-SLAM system involves combining various components into a cohesive framework that can handle the demands of real-time operation, scalability, and robustness. Key considerations include deciding when and how each component operates, structuring the compute and data-flow eficiently, and ensuring adaptability to diverse environments. As in LiDAR SLAM (cf. introductory discussion in Chapter I and Chapter 8), the problem of visual SLAM can be tackled into multiple stages (i.e., by splitting computation into a SLAM front-end and a SLAM back-end). In contrast to LiDAR-based systems, however, the 3D geometry is not directly measured since one rather observes projections of the scene irradiance onto the screen. That makes the overall estimation in visual-SLAM problems more challenging. Modern complete Visual-SLAM systems typically include three core sub-functionalities that complement each other: an odometry front-end, a mapping back-end, and a loop closure and re-localization component. In this section, we mostly provide an overview of the pipeline, while we postpone a more detailed description to Section 7.3.

## 7.2.1 Visual Odometry Front-End

The core element of a visual-SLAM system is visual odometry, which aims at estimating relative motion between consecutive camera frames. This stage provides the initial estimate for the camera’s pose. As discussed above, there are two alternative approaches to compute visual odometry: (i) Feature-based approaches split the challenge into three stages of detecting and extracting feature points, computing pairwise correspondence across images and subsequently determine the relative camera motion by minimizing the re-projection error with respect to camera motion and 3D point coordinates. The last stage is quite analogous to classical BA. (ii) Direct approaches tackle the problem in one step where a photometric loss function is directly optimized with respect to camera motion and 3D structure. They are therefore quite related to approaches of optical flow and what is sometimes called photometric BA.

At least in their naive, first formulations, feature-based methods have demonstrated a larger basin of convergence thanks to explicit data associations. To alleviate this, direct methods thus often employ a coarse-to-fine approach, i.e., start with aligning down-sampled images, or even attempt to align dense (learned) features instead of brightness or color.

## 7.2.2 Mapping Back-End

The back-end optimizes the trajectory and map using global optimization techniques like BA or pose-graph optimization. This step refines the estimates provided by the front-end and integrates observations (including the ones resulting from place recognition) into a consistent map. As a consequence, one obtains more long-term consistency, and long-range distortions are reduced.

## 7.2.3 Visual Place Recognition and Relocalization

Visual odometry is prone to drift because errors in the camera tracking will accumulate over time. In the absence of absolute positioning sensors, like GPS, one can eliminate drift and enforce global consistency by aligning the current image to previously observed images. To this end, one needs to compute correspondence across a potentially large set of images. This can be done either by an eficient matching of classical feature descriptors like SIFT, SURF or BRIEF —or by means of suitable trained neural networks, an approach that has become increasingly popular in the last years. The resulting component detects when the camera revisits a previously mapped area (loop closure detection), correcting accumulated drift and re-establishing localization when tracking fails.

## 7.2.4 Compute and Data Flow

Eficient data flow is essential for a well-performing SLAM system:

Pipeline Parallelism. Diferent components, such as tracking, mapping, and optimization, often run in parallel to maximize eficiency.

Data Sharing. Intermediate outputs, like keypoints or poses, are shared between components to minimize redundant computation.

Adaptive Scheduling. Compute-heavy tasks, like global optimization, are sched uled based on system requirements, prioritizing real-time responsiveness.

## 7.3 Visual SLAM Fundamentals

Let us now review the ingredients involved in the implementation of the visual-SLAM front-end, back-end, and visual place recognition.

## 7.3.1 Camera Model

A parameterized description of the sensor that models image formation from the observed scene should include a geometric component (also called the projection function), which describes how 3D points are mapped to 2D pixels, and a photometric component, which describes how physical light intensity (radiance) maps to pixel values.

![](images/2b7a00867b954d6477b7fdc62515f52504b3fb17ac26e9a3b33d11e7c5e48c99.jpg)  
Figure 7.1 A central requirement for visual SLAM systems is the choice of a suitable lens. Shown here are BF2M2020S23 $( 1 9 5 ^ { \circ } )$ , BF5M13720 $( 1 8 3 ^ { \circ } )$ , BM4018S118 $( 1 2 6 ^ { \circ } )$ , BM2820 $( 1 2 2 ^ { \circ } )$ , and a GoPro replacement lens (150<sup>◦</sup>). Fish-eye and wide angle lenses ofer a wider field of view, but require suitable projection models. Popular choices are the Brown-Conrady (BC) model [283], the Kannala-Brandt (KB) model [537] and the Double Sphere (DS) model [1117]. The 6-parameter DS model provides a comparable reprojection accuracy as the 8-parameter KB model while ofering around five times faster computation time for the projection function. (©2018 IEEE)

## 7.3.1.1 Geometric Camera Models

Perspective Cameras. The projection function is generically referred to as:

$$
\boldsymbol {z} = \pi (\boldsymbol {x} ^ {c}, \boldsymbol {\xi}),
$$

where $\pmb { x } ^ { c } = \left[ \pmb { x } \quad y \quad z \right] ^ { \top } \in \mathbb { R } ^ { 3 }$ is a 3D point in camera coordinates, $z = \left[ u \quad v \right] ^ { \top } \in$ $\Omega \subset \mathbb { R } ^ { 2 }$ are the coordinates of the corresponding projected 2D point in the image domain $\Omega ,$ and $\pmb { \xi } \in \mathbb { R } ^ { n }$ represents the intrinsic parameters of the camera, typically precalibrated in visual SLAM. The dimensionality of $\boldsymbol { \xi }$ depends on the used camera model.

Conversely, the unprojection operation is denoted as:

$$
\boldsymbol {x} ^ {c} = \pi^ {- 1} (\boldsymbol {z}, \boldsymbol {\xi}),
$$

which reconstructs a ray in 3D from a 2D image point z. Since the depth of the point remains unknown, the resulting $\pmb { x } ^ { c }$ is only known up to scale.

In practice, there exists a wide range of projection functions suitable for diferent lens geometries and camera types (see some, for example, in [687]). Rectilinear models (also known as pinhole- or perspective camera model) are the simplest and can be used for narrow-angle lenses without distortion

$$
\boldsymbol {\xi} _ {p} = \left[ \begin{array}{c c c c} f _ {u} & f _ {v} & u _ {0} & v _ {0} \end{array} \right] ^ {\top}, \pi_ {p} (\boldsymbol {x} ^ {c}, \boldsymbol {\xi}) = \left[ \begin{array}{c} f _ {u} \frac {x}{z} + u _ {0} \\ f _ {v} \frac {y}{z} + v _ {0} \end{array} \right], \pi_ {p} ^ {- 1} (\boldsymbol {z}, \boldsymbol {\xi}) \sim \left[ \begin{array}{c} \frac {u - u _ {0}}{f _ {u}} \\ \frac {v - v _ {0}}{f _ {v}} \\ 1 \end{array} \right],
$$

where $f _ { u }$ and $f _ { v }$ stand for the focal length in horizontal and vertical direction, in pixel units. $u _ { 0 }$ and $v _ { 0 }$ stand for the 2D coordinates of the principal point, and we assumed rectangular pixels. With typical, square pixels, we expect $f _ { u } \approx f _ { v }$

To account for some amount of lens distortion, the radial-tangential model is most commonly used:

$$
\begin{array}{c} \boldsymbol {\xi} _ {R T} = \left[ \begin{array}{c c c c} \boldsymbol {\xi} _ {p} ^ {\top} & k _ {1} & k _ {2} & p _ {1} \\ & & & p _ {2} \end{array} \right] ^ {\top}, \left[ \begin{array}{c} x ^ {\prime} \\ y ^ {\prime} \end{array} \right] = \left[ \begin{array}{c} \frac {x}{z} \\ \frac {y}{z} \end{array} \right], r = \sqrt {x ^ {\prime 2} + y ^ {\prime 2}}, \\ \left[ \begin{array}{c} x ^ {\prime \prime} \\ y ^ {\prime \prime} \end{array} \right] = \left[ \begin{array}{c} x ^ {\prime} (1 + k _ {1} r ^ {2} + k _ {2} r ^ {4}) + 2 p _ {1} x ^ {\prime} y ^ {\prime} + p _ {2} (r ^ {2} + 2 x ^ {\prime 2}) \\ y ^ {\prime} (1 + k _ {1} r ^ {2} + k _ {2} r ^ {4}) + p _ {1} (r ^ {2} + 2 y ^ {\prime 2}) + 2 p _ {2} x ^ {\prime} y ^ {\prime} \end{array} \right], \pi_ {p} (\boldsymbol {x} ^ {c}, \boldsymbol {\xi}) = \left[ \begin{array}{c} f _ {u} x ^ {\prime \prime} + u _ {0} \\ f _ {v} y ^ {\prime \prime} + v _ {0} \end{array} \right]. \end{array}
$$

Note that we cannot extract an analytical expression for the unprojection model $\pi _ { R T } ^ { - 1 } ( z , \pmb { \xi } )$ , since there is no analytical solution for $x ^ { \prime } , y ^ { \prime }$ as a function of $x ^ { \prime \prime } , y ^ { \prime \prime } \left( i . e . \right.$ undistortion). We may, however, resort to iterative approaches, $e . g .$ , the Newton-Raphson method.

Wide-angle and Fisheye Cameras. For wide-angle lenses —see Figure $7 . 1 -$ up to fields of view (FOV) of $1 8 0 ^ { \circ }$ , pinhole models with a few radial distortion coeficients typically sufice. The Brown-Conrady model [283] amounts to the radial-tangential model described above without tangential coeficients, $i . e . , \xi _ { B C } =$ $\begin{array} { r l r } { [ \pmb { \xi } _ { p } ^ { \top } } & { { } k _ { 1 } } & { k _ { 2 } ] } \end{array}$ . Note that despite the simplification relative to the radial-tangential model, no analytical undistortion exists.

For fish-eye lenses with FOVs larger than $1 8 0 ^ { \circ }$ , the Kannala-Brandt (KB) model [537] has been used, for example in [142]:

$$
\pmb {\xi} _ {K B} = \left[ \begin{array}{c c} \pmb {\xi} _ {p} ^ {\top} & \pmb {k} _ {K B} ^ {\top} \end{array} \right] ^ {\top}, \pi_ {K B} (\pmb {x} ^ {c}, \pmb {\xi}) = \left[ \begin{array}{c} f _ {u} r (\theta) \cos \psi + u _ {0} \\ f _ {v} r (\theta) \sin \psi + v _ {0} \end{array} \right] ^ {\top}.
$$

Here, the incoming ray is parametrized by the angles θ = arctan $\frac { \sqrt { x ^ { 2 } + y ^ { 2 } } } { z }$ and $\psi =$ arctan $\textstyle { \frac { y } { x } } .$ , distortion is parametrized by four coeficients $\pmb { k } _ { K B } = \left[ k _ { 1 } \quad \ldots \quad k _ { 4 } \right] ^ { \top }$ and the distortion expression is $\begin{array} { r } { r ( \theta ) = \theta + \sum _ { 1 } ^ { 4 } k _ { n } \theta ^ { 2 n + 1 } } \end{array}$

For fish-eye and wide angle lenses, a popular alternative is the Double Sphere (DS) model [1117] as it ofers a good compromise of accuracy and speed. In the DS model a point is consecutively projected onto two unit spheres with centers shifted by γ. Then, the point is projected onto the image plane using the pinhole model shifted by $\frac { \alpha } { 1 - \alpha }$ . This leads to:

$$
\pi_ {D S} (\pmb {x} ^ {c}, \pmb {\xi}) = \left[ \begin{array}{l} f _ {u} \frac {u}{\alpha d _ {2} + (1 - \alpha) (\gamma d _ {1} + z)} + c _ {u} \\ f _ {v} \frac {v}{\alpha d _ {2} + (1 - \alpha) (\gamma d _ {1} + z)} + c _ {v} \end{array} \right] ^ {\top}, \mathrm{with} \pmb {\xi} = \left[ \begin{array}{l l l l l} \pmb {\xi} _ {p} ^ {\top} & c _ {u} & c _ {v} & \gamma & \alpha \end{array} \right]. ^ {\top}
$$

As shown in [1117], this model ofers a closed-form unprojection solution. As a consequence, the 6-parameter DS model is around five times faster to compute than the 8-parameter KB model while ofering a comparable reprojection error.

## 7.3.1.2 Photometric Models

The photometric calibration maps irradiance to pixel values:

$$
I = f (E, T),
$$

where I is the pixel intensity, E is the irradiance, and T contains other camera properties that afect this mapping such as exposure time, analog and digital gain, gamma correction, de-bayering, and lens vignetting. This photometric calibration is typically only important up to scale and for methods that aim to obtain dense, textured scene representations or that rely on photo-consistency across images.

## 7.3.1.3 Time-Dependent Efects

In situations where the camera is moving while the image is taken —which is always almost the case for SLAM or Visual Odometry systems— it is important to also consider the efects of this motion on the image formation process. Most modern consumer cameras use a rolling shutter, which captures image-rows sequentially in time. In contrast, global shutter cameras, which are often used for machine perception applications, capture all image rows simultaneously.

In practice, it is advisable to either use global shutter cameras, or include the rolling shutter efect into the camera model by varying the camera pose for individual image rows. Note that this becomes significantly more practical in visual-inertial systems, where the IMU efectively measures local motion with a cameras exposure window, and thus simplifies the use and modeling of rolling shutter cameras significantly.

## 7.3.1.4 Practical Considerations

When selecting a camera model, the primary goal is to ensure that the parametric model can efectively and accurately approximate the behavior of the sensor and lens system. When using Fisheye lenses, using a spherical model is recommended

—while for rectilinear lenses, linear base-models should be used. More generally, choosing a camera model involves balancing computational eficiency against precision: using an ill-suited camera model can introduce inaccuracies and systematic biases, significantly degrading the visual SLAM system’s accuracy and robustness.

## 7.3.2 Keypoints

The success of SLAM systems in mapping environments and providing accurate localization hinges on their ability to detect, describe, and match key features in a scene –—commonly referred to as ‘keypoints’ or ‘features’. In visual SLAM, keypoint detection involves identifying salient image regions that are distinctive and repeatable, ensuring reliable re-detection from diferent viewpoints, across multiple runs, and under varying conditions. An ideal keypoint detector should maintain these properties regardless of changes in illumination, viewpoint, or occlusions. Once detected, a keypoint descriptor encodes the local appearance of the corresponding keypoint detection into compact and distinctive representations. Ideally, a descriptor should uniquely characterize a keypoint’s surroundings while remaining invariant to transformations such as lighting changes, rotation, or scale variations. This ensures both high recall –—matching the same keypoint across different conditions—– and high precision –—avoiding incorrect correspondences with similar-looking but unrelated features.

![](images/fc600d1b7744fe3046c4d3f871edb9f9d239a3e285e0d88e8e4bc725d30d8823.jpg)  
Figure 7.2 Timeline of some of the most prominent algorithms shaping the literature on visual keypoints for vision-based SLAM and image matching.

In practice, real-world challenges such as drastic illumination variations, textureless surfaces, and dynamic scenes introduce errors in detection and matching, directly impacting SLAM performance. Consequently, research has focused on designing keypoints with specific characteristics to enhance robustness. The most desirable properties include scale and rotation invariance (ensuring consistent detection despite viewpoint changes), repeatability and distinctiveness (allowing reliable re-detection and unique identification), and eficiency (enabling real-time operation under computational constraints). To meet these demands, keypoint detection and description have evolved from classical handcrafted techniques to deep-learningbased approaches more recently. The following subsections provide an overview of the most prominent keypoint methods in the literature, as illustrated in Figure 7.2, along with emerging trends in the field.

## 7.3.2.1 Classical Keypoint Detectors & Descriptors

Classical hand-crafted techniques have played a pivotal role in the evolution of feature detection and description. Among them, the Harris-Stephens keypoint detector [432], widely known as the ‘Harris corner detector,’ is one of the most influential methods. It identifies corners by analyzing the eigenvalues of the second-moment matrix within image patches, classifying a patch as a keypoint when both eigenvalues are large, indicating strong intensity variations in two orthogonal directions. The Shi-Tomasi corner detector [1004] is built on the same principle but directly uses the smaller eigenvalue for corner selection. In contrast, Harris defines a ‘cornerness’ response function to approximate the process for eficiency. While robust and computationally eficient, the Harris detector lacks scale invariance, a limitation that later methods, such as SIFT and SURF, sought to address.

A seminal milestone in keypoint detection was the introduction of the Scale Invariant Feature Transform (SIFT) [699], which set a new standard for precision and recall across challenging settings. SIFT is highly invariant to scale and rotation and partially invariant to illumination changes. It follows a structured three-stage process: (1) detecting keypoints as extrema in a scale-space Diference of Gaussians (DoG) pyramid and refining their localization through a 3D quadratic fit, (2) assigning orientation based on the dominant local image gradient, and (3) computing a 128-dimensional descriptor from a histogram of discretized gradient orientations. However, SIFT’s exceptional robustness comes at a high computational cost, making it more computationally expensive for real-time applications.

To improve eficiency, Features from Accelerated Segment Test (FAST) [946] was developed as a high-speed corner detector. It assesses pixel intensities in a circular neighborhood around the candidate keypoint location and employs an early rejection strategy to minimize computations. FAST further enhances speed using a decision tree and non-maximum suppression. However, unlike SIFT, it lacks scale invariance, making it sensitive to significant transformations. Speeded-Up Robust Features (SURF) [63] later improved eficiency by approximating SIFT using integral images and box filters instead of Gaussian derivatives, achieving a better balance between speed and robustness.

The need for fast yet robust descriptors led to the development of binary-based methods. Binary Robust Independent Elementary Features (BRIEF) [141] introduced a compact descriptor using binary intensity comparisons, enabling fast descriptor matching using the Hamming distance. While BRIEF lacks scale and rotation invariance, it demonstrated that local gradients, fundamental to SIFT’s robustness, could be efectively captured through simplified binary tests. Inspired by this success, ORB [953] was proposed, building upon FAST keypoint detection and BRIEF description, while adding scale and rotation invariance through an image pyramid and intensity centroid method. At the same time, BRISK [643] was proposed, employing FAST or Harris corners on a scale-space pyramid, and incorporating invariance to rotation changes within a binary descriptor by identifying a dominant keypoint direction similar to SIFT. While the added rotationand scale-invariance of methods such as ORB and BRISK have a small impact on the distinctiveness of the output keypoints, they have proven efective in visual SLAM and real-time robotics applications. When computational cost, however, is not a requirement (e.g., in Computer Vision applications), SIFT and SURF might still be preferable as they still ofer greater robustness under challenging lighting or perspective changes.

With several variants of these methods appearing in the literature, classical hand-crafted keypoint detection and description methods have been foundational in Computer Vision and Robotics, carefully balancing robustness, eficiency, and invariance. However, the demand for keypoints that adapt to increasingly complex, dynamic environments remains an ongoing challenge. This has driven research toward learning-based approaches, which aim to automatically optimize keypoints for real-world applications, setting the stage for the next generation of feature detection techniques.

## 7.3.2.2 Deep-Learning-based Keypoint Detection & Description

By the late 2010s, deep learning-based approaches for image keypoints began to gain traction, utilizing large-scale datasets and Convolutional Neural Networks (CNNs) to learn robust features directly from data. Unlike manually designed keypoints, these methods automatically discover and optimize feature representations, demonstrating unparalleled adaptability and accuracy in scenarios that were previously infeasible. For instance, they have shown remarkable success in detecting stable keypoints even under extreme illumination changes —an area where classical hand-crafted methods struggled. Consequently, the visual SLAM community has increasingly shifted away from traditional feature engineering, embracing data-driven representation learning for keypoint detection and description.

The Learned Invariant Feature Transform (LIFT) [1245] was one of the first deep learning-based approaches to integrate keypoint detection, orientation estimation, and descriptor computation into an end-to-end trainable pipeline. Using CNNs to extract features from small image patches, LIFT achieved greater robustness to scale, illumination, and rotation changes than classical methods. It employs a sequential learning strategy, training descriptors first, followed by orientation estimation and then keypoint detection, ensuring stable and efective feature extraction. Similarly, SuperPoint [267] introduced a self-supervised framework that detects keypoints and computes descriptors in a single forward pass, making it eficient for real-time applications. Unlike patch-based networks, SuperPoint operates on entire images and leverages homographic adaptation, a self-supervised learning technique to generate pseudo-ground truth keypoints. This approach significantly improves keypoint repeatability and descriptor quality compared to classical methods, such as SIFT, ORB, and FAST, especially under illumination changes.

Building on these advancements, HF-Net [970] introduced a hierarchical localization approach that combines global image retrieval with precise local feature matching. HF-Net improves computational eficiency while maintaining high robustness by integrating keypoint detection, local descriptors, and global descriptors into a single CNN. This architecture reduces runtime by limiting the number of images used for matching, making it particularly efective for large-scale SLAM and realtime applications, even under extreme appearance variations such as night-time scenes. HF-Net’s learned features are sparser but more discriminative than those of SuperPoint, rendering it a preferred choice for deep learning-based SLAM systems such as DX-SLAM [649].

Interestingly, D2-Net [295] introduced a describe-and-detect approach that reverses the traditional keypoint detection and descriptor extraction order. Instead of detecting keypoints first, D2-Net computes dense feature maps using a CNN and then identifies keypoints as local maxima within these maps. This method captures high-level semantic information, making it robust to extreme lighting changes and weakly textured environments. Unlike SuperPoint, which separates detection and description, D2-Net jointly optimizes both tasks, enhancing descriptor consistency. However, while this dense approach improves robustness, it is computationally more demanding than classical sparse methods. Despite this trade-of, D2-Net remains highly efective for visual localization and SFM tasks, pushing the boundaries of deep learning-based feature extraction.

## 7.3.3 Reprojection Error

The visual reprojection error measures the discrepancy between observed image points $z _ { j } \in \mathbb { R } ^ { 2 }$ and reprojected points $\pi ( \pmb { x } _ { j } ^ { c } , \pmb { \xi } ) \in \mathbb { R } ^ { 2 }$ :

$$
\boldsymbol {e} _ {\mathrm{reproj}} = \boldsymbol {z} _ {j} - \pi (\boldsymbol {x} _ {j} ^ {c}, \boldsymbol {\xi}),
$$

where $z _ { j }$ is the observed 2D image point, $\pmb { x } _ { j } ^ { c } \in \mathbb { R } ^ { 3 }$ is the 3D point in camera coordinates, $\boldsymbol { \xi }$ are the camera intrinsic calibration parameters, and π is the projection function.

Assuming the observed image points $z _ { j }$ are perturbed by Gaussian noise, the likelihood function can be expressed as:

$$
p (\boldsymbol {z} _ {j} | \boldsymbol {x} _ {j} ^ {c}, \boldsymbol {\xi}) \sim \mathcal {N} \left(\pi (\boldsymbol {x} _ {j} ^ {c}, \boldsymbol {\xi}), \boldsymbol {\Sigma} _ {j}\right),
$$

where $\Sigma _ { j }$ is the covariance of the Gaussian noise of the position of the feature in the image. In the simplest case, the noise is assumed to be isotropic and constant along the image $\Sigma _ { j } = \sigma I _ { 2 }$

Maximizing the likelihood of a set of observations is equivalent to minimizing their negative log-likelihood:

$$
\mathcal {L} = - \sum_ {j} \log p (\pmb {z} _ {j} | \pmb {x} _ {j} ^ {c}, \pmb {\xi}) = \frac {1}{2} \sum_ {j} \left\| \pmb {z} _ {j} - \pi (\pmb {x} _ {j} ^ {c}, \pmb {\xi}) \right\| _ {\pmb {\Sigma} _ {j}} ^ {2} + \mathrm{const.}
$$

Removing the constant, we get the weighted-squared reprojection error of the set of points observed in an image:

$$
E _ {\text { reproj }} = \frac {1}{2} \sum_ {j} \left\| \boldsymbol {z} _ {j} - \pi (\boldsymbol {x} _ {j} ^ {c}, \boldsymbol {\xi}) \right\| _ {\boldsymbol {\Sigma} _ {j}} ^ {2}.\tag{7.1}
$$

To handle outliers, robust kernels such as the Huber $_ { \mathrm { o r } }$ Tukey kernel are applied (cf. Chapter 3). For example, the robust reprojection error can be expressed as:

$$
E _ {\mathrm{robust}} = \frac {1}{2} \sum_ {j} \rho \left(\left\| \pmb {z} _ {j} - \pi (\pmb {x} _ {j} ^ {c}, \pmb {\xi}) \right\| _ {\pmb {\Sigma} _ {j}}\right),\tag{7.2}
$$

where $\rho ( \cdot )$ is a robust kernel function that reduces the influence of large residuals. In the following, for simplicity, we drop the dependence on the camera intrinsics $\xi .$

![](images/3ff9a4a8d2e353feb0ffa690cfe67e98cc102e95298e77aa1c08c057d26a2694.jpg)  
Figure 7.3 The feature-based visual SLAM problem: given a set of features matched in each image (left), estimate the position of their corresponding 3D points and the pose from where each image was acquired (right). Images generated with ORB-SLAM [787].

## 7.3.4 Keypoint-Based Visual SLAM

The core of feature-based visual SLAM is minimizing the reprojection error. Suppose we have a set of environment points $P _ { j } \in \mathsf { P }$ that are observed in a set of images $C _ { i } \in \mathsf { C }$ taken with a moving camera. To make notation simpler, we will just use the point and camera identifiers writing $j \in \mathsf { P }$ and $i \in \mathsf C$ . The goal of visual SLAM is to estimate the point coordinates $\pmb { x } _ { i } ^ { w } \in \mathbb { R } ^ { 3 }$ in the world reference frame ${ \mathcal { F } } ^ { w }$ , as well as the poses of the camera $\pmb { T } _ { w } ^ { i } \in \mathrm { S E } ( 3 )$ . In the monocular case, the observation of each point in each image is just the observed image coordinates $z _ { i j } \in \Omega _ { i } \subset \mathbb { R } ^ { 2 }$ (Figure 7.3).

The minimization of the reprojection error, when applied to all the points and camera poses, is known as full BA:

$$
\{\boldsymbol {T} _ {w} ^ {i   *}  ,   \boldsymbol {x} _ {j} ^ {w   *}   |   i \in \mathsf {C},   j \in \mathsf {P} \} = \underset {\boldsymbol {T} _ {w} ^ {i}  ,   \boldsymbol {x} _ {j} ^ {w}} {\arg \min} \frac {1}{2} \sum_ {i, j} \rho \left(\left\| \boldsymbol {z} _ {i j} - \pi \left(\boldsymbol {R} _ {w} ^ {i} \boldsymbol {x} _ {j} ^ {w} + \boldsymbol {t} _ {w} ^ {i}\right) \right\| _ {\boldsymbol {\Sigma} _ {i j}}\right).\tag{7.3}
$$

Typically, researchers tackle the reprojection error optimization via several advanced methods, each ofering unique trade-ofs in computational complexity and robustness:

Batch Optimization: The overall reprojection error is iteratively minimized over all observations w.r.t. all poses and landmarks, i.e., solving Equation (7.3). Popular algorithms for minimization are the Gauss-Newton (GN) and Levenberg-Marquardt (LM). This method is the gold standard in SfM, but is too expensive to run for each image in real-time SLAM.

Filtering-Based Approaches: Use sequential and causal methods like the Extended Kalman Filter (EKF) or Information Filter for real-time state estimation. They simplify the problem by keeping only the last camera pose, but unfortunately this destroys sparsity (Figure 7.4), limiting them to a few hundred features. Also, filtering methods do not re-linearize past observations, loosing accuracy.

Visual SLAM  
![](images/221409fcc710168e22d6ff29489866494d96f722859e9f7c080d86681fd9f339.jpg)  
Figure 7.4 A visual SLAM example with four camera poses and five features: Bayes network (top), and its corresponding Markov random field (middle). EKF SLAM marginalizes out past camera poses, resulting in a dense graph (lower left). Keyframe SLAM keeps just a few camera poses and discards observations from intermediate images, keeping SLAM sparsity (lower right) [1039].

Keyframe-Based Approaches: They simplify the problem by keeping just a few images, called keyframes [583]. Intermediate images and their observations are discarded for the map estimation. For the same computational efort, they can build longer and more accurate maps than filtering methods [1039].

Factor Graphs: All the above approaches can be formalized by means of factor graphs. To this end, one represents SLAM problems as graphs where nodes correspond to variables (e.g., poses, landmarks) and factors represent the reprojection errors and priors on calibration and/or camera poses, which is discussed in detail in Part I of this book.

## 7.3.5 Photometric Error and Direct Methods

The photometric error provides an alternative to the reprojection error by ofering to directly minimize the diferences in pixel intensities between observed and projected image regions. This approach is rooted in the principle of photometric consistency, which assumes that corresponding pixels in consecutive frames represent the same scene point under consistent lighting conditions.

## 7.3.6 Visual Place Recognition and Global Localization

Visual Place Recognition consists of, given a query image, finding one from the same place in a database of registered images. This is typically solved by computing a per-image descriptor $\pmb { d } = f ( I ) \in \mathbb { R } ^ { d }$ that summarizes the content of the image, and retrieving the closest one in this descriptor space via k-NN search. Galvez-Lopez and Tardos [362] introduced bag-of-words approaches that basically aggregate ORB or other descriptors by their quantization into visual clusters or “words”. While they excel at small temporal and spatial ranges, they are limited by the low invariance of hand-crafted features to variations of visual textures. For these cases, deep architectures have been proposed and trained for feature extraction and aggregation with high invariance to visual appearance changes [35, 508].

## 7.3.7 Initialization

The minimization of the reprojection error of Equation 7.3 is typically addressed by non-linear iterative optimization, which requires suficiently accurate initial guesses to converge. The initialization of the visual SLAM states in the first frames of a video sequence is hence relevant for a correct tracking, in particular for monocular setups, where the full state is not observable from a single frame.

## 7.3.8 Map Representations

Map representations are a critical aspect of visual SLAM systems, as they define how the environment is modeled and stored for navigation, mapping, and localization purposes. A well-designed map representation strikes a balance between accuracy, memory eficiency, and computational cost. We refer the reader to Chapter 5 for a broader discussion about dense map representations.

## 7.4 Further Considerations about Image Alignment and BA

Here we provide more details about image alignment (i.e., how to estimate the current camera pose from images —which is key to both odometry and loop closure detection) and briefly mention solution techniques for BA.

![](images/a87e5457596ad82ee9d38e3920fc2d7e551d2756d4671c6855438f381fb7a6a3.jpg)  
Figure 7.5 Representation of locality in ORB-SLAM [787]. The covisibility graph (left) connects keyframes that have seen at least θ points in common (in this example θ = 15) and is used for local BA. The essential graph (right) is a sparser version, that in this example connects keyframes with at least $\theta \ : = \ : 1 0 0$ points in common, and is used for pose-graph optimization during loop correction. ©2015 IEEE.

## 7.4.1 Keypoint-based Image Alignment

Although full BA (equation 7.3) is the gold standard in SfM, it is too expensive to run at frame rate, which is typically between 10 and 50 Hz. To operate in real-time, most keypoint-based visual SLAM pipelines use two key ideas:

Parallel Tracking and Mapping. splitting the SLAM process into two threads that run in parallel [583]:

A tracking thread that finds feature matchings for the current image $i \in \mathsf { C }$ and computes its camera pose, without updating the estimated map points, using pose-only BA:

$$
\boldsymbol {T} _ {w} ^ {i ^ {*}} = \underset {\boldsymbol {T} _ {w} ^ {i}} {\arg \min} \sum_ {j \in \mathsf {P}} \rho \left(\left\| \boldsymbol {z} _ {i j} - \pi \left(\boldsymbol {R} _ {w} ^ {i} \boldsymbol {x} _ {j} ^ {w} + \boldsymbol {t} _ {w} ^ {i}\right) \right\| _ {\boldsymbol {\Sigma} _ {i j}}\right).\tag{7.4}
$$

A mapping thread that solves BA only for a subset of images ${ \textsf { K } } \subset { \mathsf { C } }$ called keyframes, whose poses are the only ones that will be included in the map. In this way, BA only needs to run at keyframe rate, typically between 0.5 and 5 Hz. Keyframes can be inserted at a constant frequency, but a more sensible option is to upgrade to keyframes those frames that contain significantly new information.

Locality. when the camera is operating in a large environment, its observations have a negligible efect on the parts of the map that are far away, except in loop closure events. The usual approach is to relegate loop correction to a third thread that runs quite infrequently, and to run local BA in a window of keyframes in the mapping thread. The local window can be defined using a temporal criterion as the last k frames or keyframes, which is the usual choice in visual odometry or visualinertial SLAM systems. In visual SLAM systems, a better option is to base the local window on a covisibility criterion, for example, including in the local window keyframes that have more than θ observed points in common with the current keyframe [1038, 787] (see example in Figure 7.5). In that way, local BA can update just a set of covisible keyframes and the points observed by them (Figure 7.6):

![](images/1571b5776592d038b016412fb8a5eae23feca462d20db6e2f4eabf1a4739ddb1.jpg)  
Figure 7.6 Implementation of local BA in ORB-SLAM [787]. The local map is defined by the set of keyframes $\mathsf { K } _ { 1 }$ that contains the current keyframe k and its neighbors in the covisibility graph, and the set $\mathsf { P } _ { 1 }$ of points seen by them (in red). $\mathsf { K } _ { 2 }$ is the rest of keyframes in the map that see some point from $\mathsf { P } _ { 1 }$

$$
\{\boldsymbol{T}_{w}^{k^{*}},\boldsymbol{x}_{j}^{w^{*}}\mid k\in \mathsf{K}_{1},  j\in \mathsf{P}_{1}\} = \underset {\boldsymbol{T}_{w}^{k},\boldsymbol{x}_{j}^{w}}{\arg \min}\frac{1}{2}\sum_{\substack{i\in \mathsf{K}_{1}\cup \mathsf{K}_{2},\\ j\in \mathsf{P}_{1}}}\rho \left(\left\| \boldsymbol{z}_{ij} - \pi \left(\boldsymbol{R}_{w}^{i}\boldsymbol{x}_{j}^{w} + \boldsymbol{t}_{w}^{i}\right)\right\|_{\boldsymbol{\Sigma}_{ij}}\right).\tag{7.5}
$$

An example of a complete visual SLAM pipeline can be seen in Figure 7.7 with four threads: tracking that runs at frame rate, local mapping that runs at keyframe rate, loop closing that tries to detect loops for every keyframe and corrects them when detected, and full BA that can be run optionally to improve the map after a loop closure.

## 7.4.2 Direct Image Alignment

Direct methods such as LSD-SLAM [307] or DSO [308] typically pursue a similar pipeline of first estimating a frame-to-frame tracking and mapping and then assuring some form of global consistency. Rather than first extracting, matching and tracking points and subsequently minimizing a geometric reprojection error, however, they directly use the brightness information from the sensor and aim to compute the maximum a posteriori estimate of 3D structure and camera motion given the raw sensory data. This amounts to solving the correspondence estimation and SLAM problem jointly by minimizing a photometric loss of the form:

![](images/972a50650bdc88604f2955fe203d0385c1c74195e7ed39022b686b661e924964.jpg)  
Figure 7.7 Structure of ORB-SLAM2 system [785] showing the map, the place recognition database and its complete processing pipeline composed of four threads: tracking, local mapping, loop closing and full BA. ©2017 IEEE.

$$
E _ {p h o t o} = \sum_ {i \in \mathcal {F}} \sum_ {z \in \mathcal {P} _ {i}} \sum_ {j \in o b s (z)} \rho \Bigl (I _ {i} (z) - I _ {j} (\omega (z, d _ {z}, \boldsymbol {T} _ {j} ^ {i}) \Bigr),\tag{7.6}
$$

with respect to all camera parameters $T _ { j } ^ { i } \in \mathrm { S O } ( 3 )$ and all depth values $d _ { z } \in \mathbb { R }$ This loss assures that the colors $I _ { i }$ and $I _ { j }$ of corresponding points in all frame pairs i and $j$ are consistent. More specifically, we sum over the set of all keyframes $\mathcal { F }$ and for every point z in keyframe $\mathcal { P } _ { i }$ , we assure color consistency for all the frames obs(z) where this point is visible. The warping w takes the point z with its depth value $d _ { z }$ , transforms it from frame i to frame j with the rigid body motion $\pmb { T } _ { j } ^ { i }$ and perspectively projects it back into image $I _ { j }$

As shown in Figure 7.8, LSD-SLAM optimizes for depth maps and camera motion (tracking) in alternation and performs a pose-graph optimization (PGO) to assure global consistency of the estimated camera motion with the computed pairwise image alignments.

In contrast, DSO [308] jointly optimizes for structure and motion in the form of a photometric BA. The robust loss $\rho$ is implemented by weighted sum of squared diferences over a small patch that includes an automatic exposure time adaptation (in case the exposure time is unknown). The dependency of respective residuals is captured in form of factor graphs. And real-time performance on a CPU is achieved by limiting the optimization to a sliding window of a subset of keyframes while marginalizing out the efect of older frames. This leads to a signifcant boost in precision since it relies on a statistically optimal estimate of structure and motion given all the sensory brightness data.

![](images/1911c52f3d0018975b2654c67f1036c1f8472854f68c2a4829edf215994cfa46.jpg)  
Figure 7.8 Schematic overview of Large Scale Direct (LSD) SLAM [307] showing the three components that perform direct camera tracking, direct mapping and PGO for global consistency, all running in alternation. In contrast, Direct Sparse Odometry [308] performs the optimization of structure and motion in a single Gauss-Newton optimization in order to achieve even higher precision. Direct methods like DSO were shown to provide higher precision than keypoint-based methods because they do not perform any intermediate abstraction and can determine camera motion from even very subtle brightness variations [308] (©2017 Springer).

In real-time-capable SLAM methods, global consistency can be achieved in several steps: Firstly, one can jointly optimize over the last k keyframes as done in DSO to assure consistency over a sliding temporal window (sliding window photometric BA). Secondly, one can additionally run a PGO [553, 365] —see Figure 7.9— in order to recompute a camera trajectory that is maximally consistent with all estimated pairwise image alignments. And thirdly, one can perform an adaptive version of PGO called Pose Graph Bundle Adjustment (PGBA) [1135] which additionally incorporates the full photometric uncertainty of BA with the same computational eficiency (because only the camera poses are being updated).

## 7.4.3 Solving BA

Although BA could be solved using general variable elimination techniques as discussed in Chapter 2, there are specialized solvers that able to benefit from the sparsity structure of the problem. Figure 7.10 shows a toy example with 4 cameras and 9 points observed from them. The observation Jacobian is very sparse, as each observation $z _ { i j }$ depends only on the camera i and the point j. As a result, each observation introduces in the Hessian a diagonal block for the camera, a diagonal block for the point, and an of-diagonal camera-point block. As the number of points is typically several orders of magnitude larger than the number of cameras, a good solution is to eliminate first the points, and then solve for the cameras.

![](images/ef8ea8698f924e1a39377c8bcb393986c97fcedb4626704e3fc2a559313aff6e.jpg)  
Figure 7.9 In direct visual SLAM methods, one can perform pose-graph optimization in order to compute a trajectory that is globally consistent with all previously estimated pairwise image alignments [553]. (©2013 IEEE)

This can be done using the Schur complement. If we have a linear system where D is invertible, we can transform it by multiplying both sides on the left by a matrix:

$$
\begin{array}{c} \left( \begin{array}{c c} A & B \\ C & D \end{array} \right) \binom{x _ {1}}{x _ {2}} = \binom{b _ {1}}{b _ {2}}, \\ \left( \begin{array}{c c} I & - B D ^ {- 1} \\ 0 & I \end{array} \right) \left( \begin{array}{c c} A & B \\ C & D \end{array} \right) \binom{x _ {1}}{x _ {2}} = \left( \begin{array}{c c} I & - B D ^ {- 1} \\ 0 & I \end{array} \right) \binom{b _ {1}}{b _ {2}}, \\ \left( \begin{array}{c c} A - B D ^ {- 1} C & 0 \\ C & D \end{array} \right) \binom{x _ {1}}{x _ {2}} = \binom{b _ {1} - B D ^ {- 1} b _ {2}}{b _ {2}}, \end{array}
$$

to get a system where we can solve first $\scriptstyle { \mathbf { \mathscr { x } } } _ { 1 }$ and then $\mathbf { x } _ { 2 } { \mathrm { : } }$

$$
\begin{array}{c} \left(\boldsymbol {A} - \boldsymbol {B D} ^ {- 1} \boldsymbol {C}\right) \boldsymbol {x} _ {1} = \boldsymbol {b} _ {1} - \boldsymbol {B D} ^ {- 1} \boldsymbol {b} _ {2}, \\ \boldsymbol {D} \boldsymbol {x} _ {2} = \boldsymbol {b} _ {2} - \boldsymbol {C x} _ {1}. \end{array}
$$

The BA problem needs to solve in each iteration an equation of the form:

$$
\left( \begin{array}{c c} \mathbf {H} _ {c c} & \mathbf {H} _ {c p} \\ \mathbf {H} _ {c p} ^ {\top} & \mathbf {H} _ {p p} \end{array} \right) \binom{\boldsymbol {d} _ {c}}{\boldsymbol {d} _ {p}} = \binom{\boldsymbol {b} _ {c}}{\boldsymbol {b} _ {p}}.
$$

This can be solved in tree steps: computing the Schur complement of the points to obtain the reduced camera system, solving it for the cameras, and finally solving for the points:

$$
\begin{array}{r} \mathbf {H} _ {c c} ^ {r e d} = \mathbf {H} _ {c c} - \mathbf {H} _ {c p} \mathbf {H} _ {p p} ^ {- 1} \mathbf {H} _ {c p} ^ {\top}, \\ \mathbf {H} _ {c c} ^ {r e d} \mathbf {\Delta} d _ {c} = \mathbf {b} _ {c} - \mathbf {H} _ {c p} \mathbf {H} _ {p p} ^ {- 1} \mathbf {b} _ {p}, \\ \mathbf {H} _ {p p} \mathbf {\Delta} d _ {p} = \mathbf {b} _ {p} - \mathbf {H} _ {c p} ^ {\top} \mathbf {d} _ {c}. \end{array}\tag{7.7}
$$

As $\mathbf { H } _ { p p }$ is block diagonal, the Schur complement and the final point solution can be done very eficiently point by point. As shown in Figure 7.10 the reduced camera system is less sparse, as it contains blocks that relate pairs of cameras that have seen some point in common. In the example, there is a block between cameras 1 and 2, but not between cameras 1 and 3. In local BA the reduced camera system will be almost full and can use dense matrix solvers. In contrast, full BA has a much larger number of keyframes but there is less covisibility between them, so it can profit from a sparse solver for the reduced camera system.

## 7.4.4 Bundle Adjustment Revisited

BA has been studied for over a century and the classical approach detailed in Section 7.4.3 has been established and shown to work well in a multitude of seminal papers [1107, 14, 978]. Yet, this pipeline has two important shortcomings: Firstly, respective solutions require a suitable initialization of landmarks and camera poses. And secondly, for large-scale problems the computational and memory requirements can grow prohibitively large. In recent years, there have been a series of papers that address these shortcomings and challenge the traditional computational pipeline [469, 470, 258, 259, 1170, 1171, 1172].

The key computational bottleneck is the solution to the reduced camera system (7.7). Instead of solving this by means of an iterative conjugate gradient algorithm, Power Bundle Adjustment [1171] approximates the inverse of the Schur matrix

$$
\mathbf {H} _ {c c} ^ {r e d} = \mathbf {H} _ {c c} - \mathbf {H} _ {c p} \mathbf {H} _ {p p} ^ {- 1} \mathbf {H} _ {c p} ^ {\top},\tag{7.8}
$$

by means of a matrix power series [1171]:

$$
(\mathbf {H} _ {c c} ^ {r e d}) ^ {- 1} \approx \sum_ {i = 0} ^ {m} (\mathbf {H} _ {c c} ^ {- 1} \mathbf {H} _ {c p} \mathbf {H} _ {p p} ^ {- 1} \mathbf {H} _ {c p} ^ {\top}) ^ {i} \mathbf {H} _ {c c} ^ {- 1}.\tag{7.9}
$$

This power series provably converges to the true inverse for increasing cut of parameter m [1171]. The main advantage is that tedious matrix inversion is replaced by simple matrix multiplications, which can be done much faster and more memoryeficiently.

![](images/08f0018c9176ecdb08d325df35711cb93bef62e23750a44afb1cb82bb9b85ab8.jpg)

![](images/bf6baf771db1b2994dc9177aa7888f5c1a7aecfe3be12b426050082d9013a2a1.jpg)

![](images/64a655c637491e3e707905437905007bf26b2010ca9c1c94e7dce8cc90c00a60.jpg)

![](images/51393e5a4609686a42596e05ce9881df31459597add0a53fba604f47b6775789.jpg)  
Figure 7.10 Toy example with 4 cameras and 9 points showing in the first row the factor graph and the Jacobian of point observations. The second row shows complete Hessian, and the Hessian of the reduced camera system.

The dependency on initialization is alleviated in [469, 470] by reverting to the concept of variable projection —see Figure 7.11. To this end, the BA problem is split into two stages. In the first stage, the complicated perspective projection is replaced by a generic projective matrix such that the resulting optimization can be solved analytically for the landmarks as a function of the camera parameters. This removes the chicken-and-egg dependency between landmarks and camera poses, leading to a larger basin of attraction when optimizing the camera poses. In the second stage of projective refinement, one uses the computed solution as an initialization for the original (perspective) reconstruction. Although this strategy is computationally too demanding for more than 100 cameras, its combination with the power series approach as proposed in [1172] ofers a scalable solution for large-scale BA problems without initialization.

![](images/ac13d0f3d6a883f6474a839b6d90f6913e19a5138aad964e56317b016caa23b1.jpg)  
Figure 7.11 In a series of papers [469, 470, 1171, 1172], researchers advocate the use of variable projection methods and matrix power series in order to solve large scale bundle adjustment problems without initialization in a runtime- and memory eficient way. As shown above, camera poses and landmarks can thus be computed starting from a random initialization. (©2024 Springer)

## 7.5 Examples of Full Visual SLAM Systems

LSD-SLAM (Large-Scale Direct SLAM) [307] is a direct SLAM system that focuses on dense tracking and semi-dense mapping. It relies on photometric error minimization rather than keypoint-based methods, making it particularly efective in low-texture environments. The system operates in real-time, providing a semi-dense reconstruction of the scene and is well-suited for monocular cameras in indoor and small-scale outdoor environments.

ORB-SLAM [787, 785] is one of the most widely adopted SLAM systems due to its robustness and flexibility. It integrates keypoint-based tracking using ORB (Oriented FAST and Rotated BRIEF) descriptors, efective loop closure detection, and sparse map representation. ORB-SLAM supports monocular, stereo, and RGB-D cameras, making it highly versatile across diferent setups. It excels in scenarios requiring high accuracy and robust relocalization capabilities. In ORB-SLAM3 [142] it was extended to fisheye cameras, multimap, and visual-inertial SLAM. While originally conceived as a visual-inertial odometry system, OKVIS [645], the newest version, OKVIS2 [644], a visual-inertial SLAM system, may also be run in vision only (multi-camera) mode. Similar to the various ORB-SLAM versions, it uses keypoints and descriptors (BRISK).

Direct Sparse Odometry (DSO) [308] is a direct method for estimating 3D point cloud and camera trajectory. In contrast to LSD-SLAM camera motion and landmark points are estimated jointly in a single Gauss-Newton optimization. In order to achieve real-time performance, only the last k key frames are being updated resulting in a sliding-window photometric BA. The number k of considered keyframes provides a trade-of between speed and accuracy. Moreover, DSO makes use of a full photometric calibration with camera response function and vignette. The traditional brightness-constancy assumption is thus replaced by an irradiance-constancy assumption, i.e., the assumption that respective points in the 3D world emit the same irradiance over time. Extensions of this approach to stereo systems [1160] and omni-directional cameras [739] have been proposed. Loop closure detection has also been added to reduce drift in longer sequences [365].

While among real-time capable approaches direct methods like DSO were shown to provide more accuracy and robustness than keypoint-based methods [308], for optimal performance they typically require a good photometric calibration and a global shutter camera. The rolling shutter efect leads to geometric distortions. While these can be modeled in direct SLAM methods [980, 981], the resulting approaches are often no longer real-time capable. As a result they are better handled in keypoint-based approaches which by design minimize geometric distortions. Nevertheless, direct methods often do better on low-resolution videos where due to blurring and down-sampling it may be harder to identify reliable feature points [1227]. Furthermore, feature points are valuable for eficient re-localization and loop closuring. As a consequence, practical visual SLAM systems will often revert to hybrid approaches that combine the best of both worlds. An example of a hybrid approach is [385] where feature-based re-localization information is tightly integrated in a direct visual SLAM approach to further boost its robustness and precision.

## 7.6 Real-time Dense Reconstruction

While traditionally BA and SLAM tackle the problem of reconstructing camera motion and a sparse set of landmark points, for numerous applications ranging from augmented reality to autonomous robots one would prefer having a dense reconstruction of the observed world. To this end, a number of algorithms for realtime dense reconstruction from a monocular camera have been advocated over the years [1046, 800, 1180, 875]. Traditionally they revert to variational methods to estimate a continuous 3D structure by minimizing a loss function [1046]:

$$
\min _ {h: \Omega \to \mathbb {R}} \frac {1}{2} \sum_ {i \in \mathcal {I} (x)} \int_ {\Omega} \rho_ {i} (\boldsymbol {z}, h) \mathrm{d} \boldsymbol {z} + \lambda \int | \nabla h | \mathrm{d} ^ {2} \boldsymbol {z},\tag{7.10}
$$

with respect to a dense map $h$ that assigns a depth value to every pixel $_ z$ in the image plane $\Omega \subset \mathbb { R } ^ { 2 }$ . The residual

$$
\rho_ {i} (\boldsymbol {z}, h) = \left| I _ {i} \left(\pi \left(\boldsymbol {R} _ {w} ^ {i} \boldsymbol {x} ^ {w} (\boldsymbol {z}, h) + \boldsymbol {t} _ {w} ^ {i}\right)\right) - I _ {0} (\boldsymbol {z}) \right|,\tag{7.11}
$$

![](images/b4ff48212083f5ca39a8c97e7e02e6cb7cba84e7252e4eedf19cffe84cf21f2e.jpg)  
Figure 7.12 Dense reconstructions of the 3D world (above) can be computed in realtime from a handheld monocular camera (below) using variational methods that are efficiently parallelized on a GPU [1046]. Related approaches were proposed in [800, 1180, 875].(©2010 Springer)

enforces consistency of the brightness at pixel z in the reference image $I _ { 0 }$ to the corresponding pixels in a set of adjacent images $I _ { i }$ . The corresponding pixel is obtained by taking the 3D point ${ \pmb x } ^ { w } \left( z , h \right)$ , transforming it from the world frame by the corresponding rotation matrix $R _ { w } ^ { i } \in S O ( 3 )$ and translation vector $\pmb { t } _ { w } ^ { i } \in \mathbb { R } ^ { 3 }$ to camera i, and finally projecting it by the model π( ) into the image $I _ { i } .$

The total variation regularizer (weighted by λ) enforces spatial smoothness of the computed depth map and induces a soap-film-like fill-in for unobserved areas as shown in Figure 7.12.

## 7.7 SLAM with Depth-sensing Cameras

With the introduction of Microsoft Kinect, depth-sensing cameras became a commodity. These so-called RGB-D cameras are typically either structured-light or time-of-flight-based and provide a stream of depth images, often in conjunction with a color image stream. As such, they are conceptually between standard cameras and LiDAR sensors. Yet, in contrast to LiDAR sensors they provide an instant 2D array of depth values. Equipped with suitable algorithms, RGB-D cameras are very powerful for 3D sensing, albeit being limited to indoor applications (because the infrared-based sensing clashes with sunlight) and a certain range (typically up to around 5 meters).

![](images/209ee4a1945fbc732d52edef916ccd3e4906c197c6e6bda635853a026ed03b63.jpg)  
Figure 7.13 Dense reconstructions of a large-scale corridor scene with multiple ofices computed from a moving RGB-D camera [1030]. Using octrees [1030] or voxel hashing [808] and direct camera tracking [554], such reconstructions were demonstrated to run in real-time on a GPU [1030] and even on a tablet CPU [1031] (©IEEE)

Kinect Fusion [801] built upon previous work for range image fusion [234] to advocate a method for reconstructing camera motion and 3D structure from a moving RGB-D camera. The basic idea for fusing the various depth images into a coherent 3D reconstruction is to encode each depth image as a (projective) signed distance function $d _ { i } ( { \pmb x } )$ which for every voxel $\mathbf { \pmb { x } } \in V$ encodes the (signed) distance to the nearest surface point (along the viewing ray). Subsequently one can compute an aggregated distance function $D ( \pmb { x } )$ for all voxels as a weighted average of the individual distance functions:

$$
D (\pmb {x}) = \frac {\sum_ {i} \omega_ {i} (\pmb {x}) d _ {i} (\pmb {x})}{\sum_ {i} \omega_ {i} (\pmb {x})}.\tag{7.12}
$$

Under the assumption of Gaussian noise in the depth direction, this weighted average is nothing but the maximum-likelihood estimate of the distance function. For more robustness, one typically averages truncated signed distance functions so that each sensed surface point merely has a local impact on the reconstruction. The weights $\omega _ { i } ( { \pmb x } )$ encode the certainty of the respective surface measurements (that is sensor dependent and typically decays with distance from the object). In order to fill holes in the reconstruction, one can revert to post-processing techniques [801], or modify the above weighting scheme to ensure watertight-ness of the reconstructions [1044].

While earlier RGB-D SLAM approaches track the camera by means of aligning respective depth point clouds with the ICP algorithm [78], subsequent works advocated a direct minimization of residuals that measure color and depth consistency of two consecutive RGB-D frames $( I _ { 1 } , d _ { 1 } )$ and $\left( I _ { 2 } , d _ { 2 } \right)$ [554]:

$$
r _ {I} (\xi) = I _ {2} \left(\tau_ {g} (\boldsymbol {z})\right) - I _ {1} (\boldsymbol {z}), \quad r _ {d} = d _ {2} \left(\tau_ {g} (\boldsymbol {z})\right) - \left[ g \pi^ {- 1} \left(\boldsymbol {z}, d _ {1} (\boldsymbol {z})\right) \right] _ {z},\tag{7.13}
$$

where $g \in \mathrm { S E } ( 3 )$ denotes the desired rigid body motion, $\tau _ { g } ( z ) = \pi g \pi ^ { - 1 } ( x , d _ { 1 } ( z ) )$ denotes the induced warping between corresponding pixels, and $[ \cdot ] _ { z }$ returns the zcomponent of a point. One can then fit the distribution of all residuals $r = \left( r _ { I } , r _ { d } \right)$ with a suitable distribution and determine a maximum a posteriori estimate of the camera transformation $g = \exp ( \xi )$ by means of a coarse-to-fine Gauss-Newton optimization in $\xi \in \mathrm { s e ( 3 ) } \ [ 1 0 2 9 , 5 5 3 ]$ . Compared to the classical ICP approach, this direct tracking approach reduces the root mean square tracking error by nearly an order of magnitude on established benchmarks while being significantly faster —see [554] for details.

For mapping at larger scale, the uniform voxel representation is too memoryintensive. Therefore one typically reverts to adaptive representations using voxel hashing [808] or octrees [1030] —see Figure 7.13 and also refer to Chapter 5.

Later approaches have furthermore demonstrated scalability to larger scenes by employing a moving fusion window as in kintinuous [1183], or by including loop closures as ElasticFusion [1184] that deforms the entire dense map representation through a deformation graph.

## 7.8 Combining Vision with Other Modalities

For numerous reasons, it is advisable to combine vision with other modalities. This can provide additional robustness and precision, but it can also provide absolute scale information. Specifically, inertial sensors provide metric scale and highly reliable and robust local, relative, motion measurements. GPS/WiFi, however, may be leveraged for global (re-)localization and geo-positioning. These complementary inputs address limitations inherent to vision-only systems, making them indispensable in practical applications.

## 7.8.1 Inertial Measurement Units (IMU)

IMUs provide high-frequency measurements of angular velocity and linear acceleration, enabling precise estimation of local motion. A simple way to fuse sensory information is a loosely coupled approach where the sensory information from each sensor is independently aggregated into a pose estimate and subsequently, respective estimates are fused with a Kalman filter. While this often works fairly well in practice, for example for autonomous navigation of quadrotors [306] or nano-copters [291], it is less precise than a tightly coupled integration of sensory information.

First tightly coupled approaches leverage an EKF scheme, whereby the IMU kinematics are integrated in the prediction step, and visual keypoint measurements serve as updates –as in the seminal work of the MSCKF [777]. Alternatively, slidingwindow and batch optimizers may be formulated adopting the factor graph approach discussed in detail in Chapter 11. Figure 7.14 shows a factor graph proposed for stereo-inertial odometry [1116] that combines stereo LSD-SLAM with IMU information. Figure 7.15 shows how the tightly coupled IMU information reduces the drift leading to more crisp and precise reconstructions. This tight coupling by factor graphs is also employed by state-of-the-art keypoint-based approaches, such as ORB-SLAM3 [142], OKVIS2 [644], and many more.

![](images/39e68a558038ca7cecefb6e5cc76bb2b0a607061eca8be9771490b4408d17ec7.jpg)

Figure 7.14 The integration of multiple sensors can be elegantly performed in tightly coupled manner using factor graphs. This is an example factor graph for tightly coupled fusion of vision and IMU [1116]. It significantly increases precision and robustness of camera motion and 3D structure —see Figure 7.15. (©2016 IEEE)  
![](images/f7064f5f769bd77eeb4f7b1a43c6a22097a9a8b75f1ce2d5ee595ee3fc8b3e52.jpg)  
Figure 7.15 Tight fusion of the IMU measurements with direct image alignment results in more accurate position tracking (left) compared to the purely visual odometry system that only relies on image alignment (right). The fusion was achieved with the factor graph shown in Figure 7.14. The reconstructed pointclouds come from pure odometry, no loop closures were enforced [1116]. (©2016 IEEE)

In practice, some of the most accurate visual-inertial odometry systems start with inertial integration as basis, and use vision primarily to correct for the fastgrowing drift owing to (doubly) integrated noise and biases over time. In addition, IMUs allow to observe metric scale of motion, as well as the sensor’s orientation with respect to gravity–thus reducing the gauge freedom of the system to only 4 unknowns (global x, y, z position, as well as yaw)–compared to 7 unknowns (global x, y, z, roll, pitch, yaw, as well as scale) for vision-only systems. In many ways, IMUs are complementary to vision as a modality, and thus are ideal to combine in visual-inertial SLAM or Odometry systems.

![](images/7eeecf397c1437df32f88aba5770b4daab8cacc851f26d09d8aa97d780e5149c.jpg)  
Figure 7.16 By combining multiple sensors including stereo cameras, IMU and RTK-GPS information (left), fused in a tighly coupled manner using factor graphs, one can obtain highly accurate and robust trajectories and pointclouds, both indoor and outdoor as shown in the reconstruction of a car driving on multiple levels of a parking garage (right) [1181]. (©2020 Springer)

A multitude of visual-inertial odometry/SLAM methods have been proposed over the years, often as extensions of existing visual SLAM systems. Popular approaches include a visual-inertial version of ORB-SLAM [786], Direct Sparse Visual-Inertial Odometry [1136], VINS-Mono [896], BASALT [1118] and DM-VIO [1135]. The latter approach is a mono-inertial formulation that makes use of the concept of delayed marginalization to better capture the observability of motion in the respective sensors.

## 7.8.2 GPS and WiFi for Global Localization

While IMUs improve local motion estimation, GPS and WiFi are essential for global localization, especially in large-scale environments. In outdoor environments, GPS provides absolute position data, enabling the system to anchor the SLAM map to global coordinates. This is critical for outdoor applications like autonomous vehicles [1181, 193] —see Figure 7.16. In indoor scenarios, WiFi signals enable coarse localization where GPS signals are unavailable, complementing visual mapbased localization.

## 7.9 Further Readings & Recent Trends

Visual SLAM is an extremely active and dynamic field of research. While we tried to provide a comprehensive overview of classical visual SLAM methods, we invariably only covered a fraction of relevant work in this chapter, and we hope the reader may look into some of the many cited works for a more in-depth coverage.

Keypoint Detection and Matching. With the unprecedented invariance and adaptability of learning-based keypoints over traditional handcrafted counterparts in specific settings, deep-learning-based techniques for keypoint extraction have been transforming the field. This shift has not only enhanced keypoint detection and description but has also driven significant advancements in keypoint matching techniques. Rather than relying on manually designed heuristics for descriptor matching, as in classical methods, learning-based approaches now incorporate global spatial awareness to establish correspondences more robustly. In particular, techniques such as SuperGlue [971] and MASt3R [642] leverage Graph Neural Networks (GNNs) and Transformers to refine matches by considering the broader image structure. These methods address fundamental limitations of traditional matchers by adapting to extreme viewpoint changes, illumination variations, and occlusions, where conventional local descriptor comparisons typically struggle.

SuperGlue enhances feature matching by integrating self-attention and crossattention mechanisms, allowing it to resolve ambiguities caused by repetitive textures and occlusions. Unlike traditional keypoint matchers that operate on local descriptors alone, SuperGlue incorporates contextual information, significantly improving accuracy in both indoor and outdoor environments. Its optimal matching layer further ensures that keypoint correspondences are established robustly while allowing unmatched keypoints when necessary, making it highly efective for realworld scenarios. Similarly, MASt3R extends this idea into 3D-aware feature matching, reconstructing a 3D scene representation from two images to improve performance in textureless regions and under extreme viewpoint changes. Built upon this, MASt3R-SLAM [789] integrates these advancements into a SLAM pipeline, improving camera pose estimation, global map consistency, and loop closure strategies. By transitioning from handcrafted 2D feature matching to learning-based, context-aware, and 3D-informed approaches, these methods mark a paradigm shift in visual SLAM, enhancing scene understanding, tracking robustness, and long-term stability in complex environments.

Overall, the evolution of keypoint extraction for image matching and visual SLAM has been remarkable from traditional hand-crafted methods to deep-learning based approaches that promise greater robustness in challenging conditions. While classical methods remain very relevant to date due to their eficiency and interpretability, they struggle in scenarios with textureless surfaces, extreme viewpoint changes, and illumination variations. Deep learning has addressed these limitations, driving research towards more adaptive and context-aware keypoint detection and matching techniques. However, challenges remain in balancing computational eficiency with real-time performance, particularly on resource-constrained platforms, and ensuring the availability of diverse, unbiased training datasets. Future research will likely focus on optimizing these techniques to maximize both accuracy and eficiency, making them more viable for large-scale real-world applications.

Other Frontiers. We emphasize that there have been many exciting developments in visual SLAM in the recent past. Among other developments, there are generalizations of visual SLAM to dynamic environments with moving and potentially deformable objects. In addition, learning-based approaches to visual SLAM are becoming increasingly popular: in a multitude of publications, more and more components of the classical visual SLAM pipeline (feature extraction, correspondence estimation, image alignment, camera tracking, BA, dense reconstruction, etc.) are being enhanced or replaced by learning-based formulations. All these ideas will be discussed in more detail in Part III of this handbook.

LiDAR SLAM Jens Behley, Maurice Fallon, Shibo Zhao, Giseop Kim Ji Zhang, Fu Zhang, and Ayoung Kim

Along with cameras, LiDAR sensors are one of the most popular sensing modalities used in robotics. LiDAR is a technology that uses a laser to actively transmit laser light pulses and then measures the time delay in those pulses reflecting of of surfaces and returning to a detector. In doing so it directly measures the distance to those surfaces. A LiDAR sensor can be used to perceive the structure of its surrounding environment and also to estimate the motion or ego-location of the sensor.

The development of LiDAR technology began in the 1960s and 1970s with stationary systems primarily used for applications such as atmospheric research, topographic mapping, and military applications. These early LiDAR systems were bulky and expensive, making them unsuitable for mobile applications. In the 1980s, advancements in laser technology and computing power allowed for more compact and afordable LiDAR systems. These systems were still primarily stationary and used in applications like terrain mapping and environmental monitoring [248, 683]. In the 1990s, the integration of LiDAR sensors with Global Positioning System (GPS) and IMUs began to enable the first mobile LiDAR mapping systems. These systems were often mounted on vehicles or aircraft to create detailed 3D maps of large areas [499]. In the late 1990s, researchers began exploring the use of LiDAR for realtime SLAM in robotics. 2D LiDARs were instrumental in driving and catalyzing progress in probabilistic mapping and SLAM in the early 2000s, and 3D LiDARs became even more prominent in robotics and self-driving vehicles with the DARPA Grand Challenge in 2004 and the DARPA Urban Challenge in 2007.

In this chapter, we start by briefly reviewing the underlying technology within diferent types of LiDAR sensors (Section 8.1). Then, we discuss LiDAR odometry (Section 8.2) and LiDAR-based place recognition (Section 8.3). Moreover, we discuss the integration of odometry and place recognition into full LiDAR-SLAM systems (Section 8.4). Finally, we provide pointers to further readings and discuss new trends (Section 8.5).

![](images/870e7b9e4e8d5d512224f9f49983d6cac81e8dc55b560f24d1777cb42daf1531.jpg)  
(a) 2D LiDAR

![](images/50ca1739cdfe3c75665eca1e186e9ab1e4785ebfe76f1dd2e239708a622a4ebc.jpg)  
(b) 3D Rotating LiDAR

![](images/af00466d333cfaf3a3615d3e3bf96313bcf79d0e7b4600372630ea12bc68f578.jpg)  
(c) Spiral Scanning  
Figure 8.1 Common LiDAR sensor types and their beam patterns. (a) A standard 2D LiDAR sensor with a rotating transmitter mirror (yellow). The encoder disk (blue) is used to measure the rotation angle of the mirror. (b) An example of a mechanical multibeam LiDAR emitting multiple laser beams from the diferent emitters (yellow), which are then detected using the detectors (blue). The entire sensor head rotates to generate a 360<sup>◦</sup> horizontal field of view. (c) A macroscopic steered Risley prism LiDAR with an spiral scanning pattern which only measures in the direction of the lens window (cyan). Due to the spiral pattern it can produce denser point clouds over time.

## 8.1 LiDAR Sensing Preliminaries and Categorization

By rotating the laser emitter and detector within a LiDAR sensor in one or two axes, it is possible to build up a detailed point cloud of the environment around the sensor. The most basic principle is time-of-flight (TOF), which employs laser pulses to infer distances using the measured time it takes for emitted light pulses to return to the detector. TOF LiDAR sensors can capture high-resolution measurements but are sensitive to external light, which reduces the signal-to-noise ratio (SNR) [598] and therefore the accuracy and frequency of measurements. Besides TOF, the techniques of Amplitude Modulated Continuous Wave (AMCW) and Frequency Modulated Continuous Wave (FMCW), originally developed for radar sensors have been adopted for LiDARs.

The classes of LiDAR sensors are categorized by their sensing mechanisms [935] and can be classified into mechanical LiDARs, scanning solid-state LiDARs, flash LiDARs, and sensors using macroscopic scanning Risley prisms. Among these, we will examine two commonly utilized types: 2D/3D mechanical LiDARs and macroscopic scanning LiDARs.

Mechanical LiDARs are the most common category of LiDAR. They use a rotating assembly to direct the laser beam but face limitations due to mechanical wear and lower rates of data acquisition. The simplest 2D mechanical LiDAR sensors use a rotating mirror to direct a single laser beam and to measure distances, as shown in Figure 8.1a. By using an encoder disk, the angular orientation of the LiDAR beam can be measured and paired with the range measurement to produce a 2D profile measurement. By its nature, this LiDAR can only scan in an individual 2D plane.

The demand for a single sensor that can scan in full 3D was motivated by the

![](images/d5d3296b09c757d26b906f1c963c7c62cb8e39e58ac20823c8102fa50e3b145b.jpg)  
Figure 8.2 Example of a single multi-beam LiDAR scan and a corresponding image. The scan is from a 64-beam Hesai QT64 scanner with 104<sup>◦</sup> vertical field-of-view. Note the individual scanning lines and also how dense the point cloud is close to the device (the colored axis in the 3D view) and much sparser the cloud is further away.

DARPA Grand Challenges (an early self-driving car competition) in the 2000’s and led to the development of the pioneering Velodyne HDL-64E sensor. It was used by most of the teams [1115, 772, 536] in the challenge. Within a 3D mechanical LiDAR, multiple laser emitters are mounted on a single rotating mechanism to capture individual range measurements pointing in diferent elevation angles as the entire mechanism rotates through 360 degrees in the azimuthal axis as illustrated in Figure 8.1(b). The resulting point cloud can capture a highly detailed 3D depiction of objects and the sensor’s surroundings as shown in Figure 8.2, as discussed in various studies [1178, 557, 935]. Subsequently, the technology has evolved —with the size and price of LiDAR sensors dropping significantly.

A wider variety of prototype sensors have emerged more recently drawing on a variety of physical properties including solid-state LiDARs, Risley prisms, and polygonal mirrors. In contrast with traditional scanning laser, many of these sensors use Micro-electromechanical (MEMS) [467] mirror technology or optical phased arrays (OPA) [446] to avoid, or at least minimize, mechanical rotation. This is important because it can enhance the LiDAR’s lifespan and reliability in environmental mapping, since fewer mechanical parts need to be actively actuated which reduce the mechanical wear.

A notable advancement is the use of Risley prisms [668] which enables rapid, controlled beam steering with much smaller amount of physical movement. This innovation results in a more compact sensor, albeit with more limited FOV currently.

All the aforementioned sensors produce sets of individual range measurements with intensity (often also called remission) value for each measurement, i.e., how much of the LiDAR beam is reflected. The range measurements with associated angles of the individual beams can be then converted into a 2D or 3D point cloud. Yet, more recent advances have introduced additional measurement capabilities beyond the default range measurement. For instance, FMCW LiDARs continuously project light with varying frequency and can measure the relative velocity of the object the beam hits using detected frequency shifts. This is similar to how FMCW is used in radar. This approach is useful in dynamic environments or challenging scenarios [1201], although it tends to be more complex and costly compared to other variants. Another innovative sensing mechanism is flash LiDAR, which can provide ambient channels resembling photometric measurements, similar to those obtained by cameras. These technologies, with their diferent characteristics of power consumption, weight and cost, provide a variety of options when carrying out LiDAR odometry and SLAM for diferent applications.

## 8.2 LiDAR Odometry

The first building block of LiDAR SLAM is LiDAR odometry. The goal of LiDAR odometry is to estimate the incremental ego-motion of a robot or vehicle in realtime given a LiDAR scan and past observations, i.e., a single scan or multiple scans aggregated into a local map. Here, the term scan refers to a single sweep or cycle of data collected by the LiDAR sensor. More specifically, a scan typically represents one complete rotation or one full sweep of the sensor providing a contextual snapshot of the surrounding environment at a specific time. Thus, scans are often time-stamped, allowing them to be ordered and processed as sequential observations.

At the heart of LiDAR odometry lies the technique of scan registration, also referred to as scan matching. Scan registration involves finely aligning a pair of scans to estimate the precise relative transformation between the scans. A scan is efectively a set of points or a point cloud. In the literature, several important algorithms have been proposed for point cloud registration, including Horn’s method [477] and ICP. Although Horn’s method provides a closed-form solution, it requires pre-determined correspondences, which limits its applicability in real-world scenarios. In contrast, the ICP algorithm [79, 959] does not require prior knowledge of correspondences and has therefore become a more popular and fundamental technique for determining the relative transformation between point clouds. We will discuss ICP further in the next sections.

The original development of LiDAR SLAM can be traced back to the seminal work of Lu and Milios [701], who pioneered the concept of globally consistent 2D range scan registration by introducing the idea of a network of poses —a concept closely resembling the modern pose-graph approach. This work also laid the groundwork for LiDAR odometry by defining the fundamentals of 2D scan registration. Key contributions to the probabilistic framing of scan-to-scan matching were made by works including [826, 596]. While points and lines are common choices in 2D scan matching, Olson [825] introduced an impactful approach using correlation techniques for real-time registration.

Early extensions to 3D LiDAR built on these 2D scanning techniques by actively moving the sensor in a nodding [433] or rotating manner [100], or by passively using the motion of a human carrier [102] or vehicle [100] to accumulate denser 3D point clouds. These 3D point clouds enabled 3D scan matching for LiDAR SLAM but introduced significant computational challenges due to the increased data size. Addressing these challenges, LOAM [1269] demonstrated real-time scan matching capabilities forming the basis for follow-up methods in LiDAR odometry and SLAM.

## 8.2.1 Foundations of Scan Registration

Scan registration is a fundamental component in LiDAR odometry and mapping systems. It involves the alignment of two scans to achieve an accurate alignment and mapping. The goal is to find the transformation, i.e., rotation $R \in \mathrm { S O ( 3 ) }$ and translation $\pm \in \mathbb { R } ^ { 3 }$ , that can best bring one of the scans (potentially a recent scan from a sensor) into alignment with the other $( e . g .$ , a previous scan or a local map). In doing so, this process also yields the relative position from which the scans were taken. A large body of techniques and algorithms have been developed to perform scan registration with high accuracy, robustness, and low computational cost.

Iterative Closest Point and Its Variants. As introduced in Chapter 5, a point cloud is defined to be a set of points in a three-dimensional coordinate system, represented mathematically as $\mathsf { P } = \{ \pmb { p } _ { i } \in \mathbb { R } ^ { 3 } \mid i = 1 , 2 , \ldots , N \}$ , where each $\pmb { p } _ { i } = ( x _ { i } , y _ { i } , z _ { i } )$ denotes the 3D coordinates of a point. For scan registration, the ICP algorithm [79] minimizes the total registration error between two point clouds P and Q. Let us denote P as a source and Q as a target point cloud.

ICP iteratively determines transformations $R ^ { k } , t ^ { k }$ for an optimization iteration k that minimize the total registration error, which is measured by diferent distance metrics d and is given by

$$
\boldsymbol {R} ^ {k}, \boldsymbol {t} ^ {k} = \underset {\boldsymbol {R}, \boldsymbol {t}} {\arg \min} \sum_ {(\boldsymbol {p}, \boldsymbol {q}) \in \mathbb {C}} d (\boldsymbol {p} _ {i}, \boldsymbol {R} \boldsymbol {q} _ {i} + \boldsymbol {t}),\tag{8.1}
$$

where the set of correspondences C between the source point cloud P and target point cloud Q is given by

$$
C = \left\{\left(\boldsymbol {p}, \boldsymbol {q}\right) \mid \boldsymbol {p} \in P, \boldsymbol {q} \in Q \right\}.\tag{8.2}
$$

In the ICP algorithm, determining the transformation between the source and target is achieved iteratively by recomputing for each iteration k a new set of correspondences C based on the last transformation at iteration $k - 1$ given by rotation $\pmb { R } ^ { k - 1 }$ and translation $t ^ { k - 1 }$

To minimize the total registration error in (8.1), two aspects must be specified:

1 What geometric relation is used to define the distance measure $d ( \cdot ) ?$ The aim is to align two point clouds as tightly as possible, and this tightness cannot be determined without defining a distance metric.

![](images/5b7c3e986fa50b98e3cf906ec248e3445ff50e7c22e228e0b8c449db45923ffd.jpg)  
Figure 8.3 Typical distance metrics used in ICP. (a) Point-to-point distance is as straightforward as the Euclidean distance between two points. (b) and (c) The point-to-higher-level feature $( e . g .$ , line or plane) is calculated as the shortest distance to the reconstructed line or plane using the target points.

2 How is the correspondence set C, used for minimization, determined? This involves identifying the corresponding target point $q \in \mathsf { Q }$ for each source point $p \in \mathsf { P }$

Common approaches used to address these two aspects are discussed below.

## 8.2.1.1 Distance Measure in Registration Residual

The first question above involves deciding which geometric elements to use for defining the residual, possibly going beyond simple point-to-point distances. Typically, points, lines, and planes are the most common geometric elements used to define the distance in equation (8.1), as summarized in Figure 8.3.

Point-to-point ICP is the most basic approach and it minimizes the Euclidean distance between corresponding points in the two point clouds. Pioneering works in ICP by Zhang [1281] and Besl and McKay [79] formulated shape $( e . g .$ , curves and surfaces) matching as a point matching problem by representing shapes as sets of points. This point-to-point cost is straightforward and simple, but can be sensitive to noise, outliers, and sparsity of the measured points. Distances to lines are also commonly used as an error measure. The point-to-line distance measures points in one point cloud and lines (formed by connecting points) in the other point cloud. It can provide better results in structured environments with linear features. By exploiting higher-level geometric features we can go further. We can measure the distance between points in one point cloud and planes (local surfaces) in the other point cloud. This approach is more robust to noise and can achieve higher accuracy in environments with planar surfaces.

Extending from these basic geometries, other ICP variants introduce diferent distance metrics. Techniques which employ multi-distance metrics [839], continuoustime formulation [256], and adaptive thresholds [1133] are more some of the more recent ICP advances. Other methods [985, 591] opted to evaluate diferences in a probability distribution of a local neighborhood than using Euclidean distances.

Another well-known distribution-based matching is the normal distributions transform (NDT) [82]. NDT divides an input point cloud into a set of voxels and fits a normal distribution to the points in each voxel (see Chapter 5.3.2.2 for more details). Instead of incurring the cost of determining nearest neighbor associations, it takes advantage of voxelization to carry out a distribution-to-distribution matching process. This process can take advantage of a smoother and more robust registration cost surface, especially in complex environments.

## 8.2.1.2 Determining Correspondences

The second core design decision of common ICP algorithms is data association or correspondence search between the source P and the target $\mathsf { Q }$

In the most basic form, determining correspondences between P and Q can be achieved geometrically by finding the nearest neighbor of a point $p \in \mathsf { P }$ in the target Q, where we use the iteratively updated transformation $( R ^ { k - 1 } , t ^ { k - 1 } )$ , which is given by:

$$
C = \left\{\left(\boldsymbol {p}, \boldsymbol {q}\right) \mid \boldsymbol {p} \in P, \boldsymbol {q} = \underset {\boldsymbol {q} ^ {\prime} \in Q} {\arg \min} | | \boldsymbol {p} - \left(\boldsymbol {R} ^ {k - 1} \boldsymbol {q} ^ {\prime} + \boldsymbol {t} ^ {k - 1}\right) | | _ {2} \right\}\tag{8.3}
$$

However, this is typically an expensive operation when computed over a large point cloud with thousands of points.

To make real-time operation of LiDAR odometry possible, there are two common strategies used to reduce the time for correspondence search: (1) reducing the set of potential candidates for a correspondence, or (2) employing a diferent search strategy than exact distance-based neighbor search to more quickly find potential candidates.

Several ICP variants used in popular LiDAR odometry systems [1269, 839, 999] employ the first strategy to reduce the set of potential correspondences by building maps with reduced candidate sets, $\mathsf { P } ^ { \prime } \subset \mathsf { Q }$ and $\mathsf { Q } ^ { \prime } \subset \mathsf { Q }$ by determining points that fulfill certain geometric criteria. The criteria used include determining points lying on edges or surfaces [1269, 839], removing less descriptive points that correspond to the ground plane [998], or downsampling of the target scan [1133, 256]. While these strategies certainly speed up the correspondence search, they have the potential drawback of removing true correspondences from the target Q.

In contrast, the second strategy employs search structures with eficient approximations which enable faster correspondence searches even though they may not always yield the exact nearest neighbor. In this direction, a common strategy is to use projective neighbor search in range images [66] or leveraging voxel grids for approximate neighbor search [1269, 256, 1133]. Furthermore, while we covered here the pure geometric correspondence search in Euclidean space, it is also possible to use alternative distance metrics, as well as to apply projections into a feature space [839] to identify correspondences.

In the following sections, we will discuss other components of a LiDAR odometry system that are integrated around the core ICP component to build out a complete scan registration system which can align a sequence of scan observations so as to estimate the relative motion of a robot.

![](images/4d9bfd9ea3196ac247d22eb788a86b2f03a6e566e8fc4a0bd48664790df19f0e.jpg)  
Figure 8.4 Components of a LiDAR odometry pipeline: Given a current scan, (1) motion compensation accounts for the motion of the sensor during the scan process resulting in a undistorted scan. Then, (2) correspondences between the undistorted scan and the previous scans, either a single scan or aggregated scans in a local map, are determined. Finally, (3) the relative pose estimate is determined via a scan registration to estimate the relative pose of the current scan. These steps are iterative with the correspondence set refined based on the intermediate relative pose estimates. After convergence, the final pose estimate is the output of the LiDAR odometry system.

## 8.2.2 Common Components for LiDAR Odometry

This section outlines the key modules involved in a LiDAR odometry system: point cloud motion compensation, identifying correspondences, and pose estimation via scan registration. Point cloud motion compensation addresses the distortion caused by the motion of the LiDAR during scan acquisition. Correspondence search identifies matching points between consecutive scans that are useful for matching and scan registration. Finally, the pose estimation module estimates the sensor’s motion since the previous registration. Registration can be carried out either scan-to-scan between consecutive scans or scan-to-map with respect to a local map. Together, these modules ensure accurate and reliable LiDAR odometry. Figure 8.4 shows the interplay between the diferent components in a common LiDAR odometry pipeline.

## 8.2.2.1 Point Cloud Motion Distortion Compensation

Motion distortion in LiDAR odometry occurs because a LiDAR sensor captures a scan over a period of continuous motion.<sup>1</sup> Due to this movement during the scan period, the sensor will emit laser pulses and receive ranging returns from slightly diferent times and positions. This means that a single scan does not represent a static snapshot of the surroundings at a single position but instead a set of points each captured from a slightly diferent scanning position. This continuous movement during the scanning process will lead to inaccuracies and in turn a distorted point cloud if left uncorrected.<sup>2</sup> For example, Figure 8.5 clearly illustrates a distorted point cloud sample and highlights the need for proper motion compensation. As can be seen, undistorting the point cloud to account for the motion of the LiDAR sensors is an important pre-processing step in LiDAR odometry which can improve accuracy and robustness. Compensation methods have been developed to correct this efect including a constant-velocity model, continuous-time trajectory optimization, and using IMU measurements.

Constant-Velocity Model. The constant velocity model assumes that the robot maintains the same translational and rotational velocities estimated during the previous time step. As this model does not require any additional sensors, it can be widely used in simpler LiDAR odometry systems [256, 1133], however, the constant velocity assumption is inherently less accurate when the motion contains high-frequency motions.

Continuous-Time Trajectory Optimization. Another widely used approach for motion compensation is continuous-time trajectory optimization techniques using splines [282, 715] and the GP [50] (cf. Chapter 2). Continuous-time trajectories allow pose estimates to be made at any time instant without relying on linear interpolation. They can be used to remove the distortion of each individual point, however, conventional continuous-time trajectory optimization is time-consuming and often implemented ofline [715].

IMU-based Motion Compensation. IMU measurements directly measure high-frequency motion with gyroscopes and accelerometers. They can be integrated over the scan period as an efective approach for motion compensation [1269, 999]. IMU-based motion compensation pre-integrates the LiDAR pose using the most recent IMU data and then uses that predicted trajectory to rectify for point distortion. Thanks to the high frequency of IMU measurements (e.g., 200 Hz), IMU-based motion compensation is highly efective for jerky robot motions and is now the defacto standard for most robot platforms. Nonetheless, this method needs to be used carefully because IMU measurement noise, bias, estimation errors, and poor clock synchronization can cause this approach to under-perform other simpler methods.

Point-wise Registration. Being firstly proposed in Point-LIO [442], this pointwise approach is fundamentally diferent from existing scan-based LiDAR odometry frameworks. In this framework, the state is updated by processing each LiDAR point when it is received, rather than accumulating a complete scan. As a result of this design, the proposed method does not sufer from intra-scan motion distortion.

## 8.2.2.2 Feature-based LiDAR Odometry

Once motion distortion is corrected, point correspondences can be established. Similar to visual SLAM (see Chapter 7), leveraging features for scan association is well studied. A feature-based approach allows eficient representation of the scan by processing only a small number of features extracted from the point cloud. Correspondence matching and residual computation can also be performed at the feature level, substantially reducing the overall computational cost.

![](images/8587ef8edda1ce234f8e06320e5661f420ef1cef9c69ac12153f0a2308c74552.jpg)

(a) LiDAR-camera overlay - without motion undistortion  
![](images/2185bef4395a451de3f67285ced75e857180f2743b132376178df19cafc901af.jpg)  
(b) LiDAR-camera overlay - with motion undistortion  
Figure 8.5 LiDAR point cloud overlaid on a camera image. A slight misalignment at the start of the sweep (‘b’) worsens significantly by the end of the LiDAR sweep (‘a’). At the bottom, more consistent overlays are observed for both the start (‘d’) and end (‘c’) after motion compensation. From [1072].

Low-level Features. Lines and planes are the most commonly used features in practice. In this line of study, the well-known LOAM algorithm [1269] was a significant breakthrough in LiDAR SLAM. LOAM uses a low-level detector to efficiently identify mid-level features that can contribute to scan registration. Points with high or low curvature are identified to detect edges and planes. The curvature of each point is calculated by analyzing the diferences between the point and its neighbors. High curvature points are marked as edge features, while low curvature points are identified as planar features. Not all points are used as features; the algorithm selects a subset of the most significant edge and plane points to reduce computational complexity while maintaining accuracy.

Principal component analysis can also be employed to identify the principal directions of a local neighborhood of a point for efective feature detection [67, 753]. By calculating the eigenvalues and the corresponding eigenvectors of the covariance matrix of a point and its neighbors, the main axes of variation can be determined. This analysis allows us to discern the geometric properties of the points. Points with one dominant eigenvalue can then be classified as edge points, indicating sharp transitions or boundaries. In contrast, points with two similar eigenvalues can be identified as being from planar regions, representing flat surfaces within the point cloud. This enables diferentiation between edge and planar features, enhancing the accuracy of LiDAR odometry by considering the geometric properties.

High-level Features. In LiDAR odometry, high-level features such as semantic, surfel, and intensity features can also play an important role in enhancing the accuracy and robustness of the system. These features provide richer information about the environment compared to low-level features, facilitating better scene understanding and more accurate mapping.

Semantic Features: Semantic features involve the use of machine learning and deep learning techniques to classify and label the LiDAR point cloud into object categories such as vehicles, pedestrians, buildings and vegetation —typically to distinguish between dynamic and static objects. It can improve the reliability of odometry by focusing on stable landmarks [187].

Surfels Features: Surfels are small disk-like representations of the surface geometry of a point cloud (see Chapter 5.3.2.2). An ellipsoid disk can be fit to a set of points with the ellipsoid’s principal semi-axes lengths determined by the eigenvalues of the covariance matrix of nearby points. This type of surface representation can then be used to compute point-to-plane distances during scan registration [846, 66, 903].

Intensity Features: Intensity features [277, 415] refer to the reflectivity or intensity values of the LiDAR returns. These values provide additional information about the material properties and surface characteristics of objects in the environment. They improve the robustness of feature matching by providing an additional dimension of information, which can be crucial in challenging scenarios such as structure-less environments.

## 8.2.2.3 Direct Point-wise LiDAR Odometry

A problem with the feature-based approach described above, is that it tends to discard subtle contributions from isolated points which do not clearly correspond to planes or edges. This is particularly a problem when mapping unstructured environment with bushes or branches in natural environments. It also requires tuning of hand-engineered feature detectors when moving from one sensor to another. Additionally, the number of points to be processed by the backend typically scales linearly with the size of the LiDAR scan. The computational cost of this approach can become impractical when working with modern 64 or 128 beam sensors.

Alternatively, one can use the points directly —without extracting mid-level features. Similar to the direct methods in visual SLAM, we can directly align points in an ICP-like manner. However, due to the high computational cost during pointwise correspondence matching, this direct methods were not favored in the early development of LiDAR SLAM.

Paving the way for direct methods, Zhou et al. [1295] made it practical to use direct methods by speeding up the nearest neighbor search with a GPU-accelerated KD-tree implementation. Later the direct method Fast-LIO2 by Xu et al. [1211] demonstrated highly accurate frame-rate odometry without sufering from a correspondence search bottleneck when the map grows large using an extension, the so-called incremental KD-tree (iKD-tree). The iKD-tree can adapt to the distribution of points by occasionally rebalancing itself to allow for eficient add, remove and query operations, which avoids rebuilding the tree for each added scan.

## 8.2.2.4 Local Mapping and Pose Estimation

Once reliable correspondences have been obtained, the next step is to achieve consistent registration of the consecutive scans. As mentioned before, incremental pose estimation in LiDAR odometry is achieved, in most cases, via a scan registration with a variant of ICP (see Section 8.2.1).

Initially methods focused on scan-to-scan odometry and sought to achieve full frequency (10 Hz) output while treating each consecutive scan registration operation as being independent. However, as individual LiDAR scans can be relatively sparse, many points will have unsuitable correspondences if the reference scan is simply the previous scan. This will result in registration errors accumulating and an inconsistent overall map.

A more modern approach is to build a detailed and accurate local map around the sensor which is known as scan-to-map odometry. This paradigm has been successfully used in 2D LiDAR SLAM [458], 3D LiDAR odometry [1211, 256, 1133], and 3D LiDAR SLAM systems [839, 66, 846] and has been seen to reduce overall drift rates.

The motion prediction from either scan-to-scan odometry or IMU pre-integration can be used to pre-align the incoming scan before a fine registration to the persistent local map is carried out. The local map which becomes much dense than an individual scan results in much more suitable inlier associations. After registration, the incoming scan will be added to this local map.

Earlier LiDAR odometry approaches [1269] needed to resort to interleaving scanto-scan odometry at a high frequency with scan-to-local-map odometry at lower frequency due to compute restrictions. More modern LiDAR odometry approaches [256, 1133] now use a single-stage scan-to-map alignment with direct point-wise correspondences enabled by a voxelized local map representation.

Finally, a quite diferent approach to LiDAR odometry is to use deep learning. Efforts to learn ego-motion directly from LiDAR measurements has resulted in some promising works. Early studies employed supervised learning using ground-truth labels [655], and this line of work has been extended to unsupervised learning methods [210]. While the performance of deep LiDAR methods are generally promising, concerns have been raised regarding their generalization capabilities.

![](images/7d22ccac97b810c9d5473671fd2ca8c94856b1bd271eac8b587b67e531a376c4.jpg)

![](images/239cf867821f6749529cfde6e749eb8dca130a863e14dde869621465f72c13a4.jpg)  
(a) Place A

![](images/ac1d20678d75f061e5c806c0b3e1c5505c400a75c0661ce26fd0ef31049e661b.jpg)

![](images/2861c6ac16303733ee100e61e642d8a37cf28bdb19d42b5b5674c72d43de32b8.jpg)  
(b) Place B  
Figure 8.6 Despite the large visual diferences between Place A and Place B in the RGB images (aerial view and robot’s front-looking view), their corresponding LiDAR scans exhibit structurally similar patterns. This causes structural perceptual aliasing, where distinct places appear similar to LiDAR-based place recognition algorithms due to shared road topology and surrounding structures. This example is captured from the SNU Afternoon sequence of the STheReO dataset [1262].

## 8.2.3 Summary of LiDAR Odometry

To summarize, common 3D LiDAR odometry algorithms can produce highly accurate and robust motion estimates by iteratively aligning incoming scans to a running local map —often with the support of IMU measurements or motion models to correct for motion distortion of the scan. Resulting systems can achieve drift rates in the order of 1 m per 1000 m traveled —but the performance is highly dependent on the environment around the robot and the level of dynamics present in the scene and the dynamics of the sensor itself. Accounting for this remaining amount of small drift is a key aspect of LiDAR SLAM with place recognition being a key component of such a system.

## 8.3 LiDAR Place Recognition

Place recognition systems seek to identify places that have been previously visited by a robot/sensor. It is a key capability in LiDAR SLAM and also enables related problems, including multi-session SLAM as well as global localization in a prior map. Unlike visual data, LiDAR data allows a robot to obtain consistent metric 3D information about the surrounding environment. This capability ensures that LiDAR is less afected by lighting condition changes than conventional cameras. However, despite this advantage over visual localization, the nature of LiDAR sensing presents unique challenges for LiDAR place recognition. For example:

Sparse Data. In contrast to visual measurements, where pixels are dense (e.g., megapixel cameras) and organized (i.e., structured into a regular 2D grid), the spacing and local density of points captured by conventional LiDAR sensors varies depending on the type of sensor $( e . g .$ ., the number of beams) and the sensing range (e.g., points farther away are sparser). The resolution of point clouds is typically much lower than camera images. Because of these limitations, LiDAR place recognition typically does not rely on local keypoint descriptors, where each point has its own feature descriptor. To address the lack of structure, approaches identify semantically meaningful point cloud segments [285, 1274] or compute global descriptions [568, 1212] (i.e., a single representative descriptor for a scan). Recently, with the advancement of deep learning, learning to determine robust local keypoint descriptors has also been actively studied [156]. Seminal papers and diferent paradigms in the area of LiDAR-based place recognition will be revisited in more detail in Section 8.3.2.

Structural aliasing. The second dificulty that LiDAR-based place recognition systems face is structural repetition in man-made environments such as long corridors or indistinguishable structures on highways. Consider the corridors on each floor of a regular modern ofice building. Using a camera, visual place recognition might be able to identify unique or descriptive visual texture (e.g., pictures, posters or decorations) on otherwise identical corridor walls. However, it is very dificult to distinguish the specific floor using only a single scan obtained in the corridor. A similar challenge arises in outdoor environments, where perceptual aliasing can occur between visually distinct places that share similar structural layouts in LiDAR scans, as shown in Figure 8.6. Global LiDAR descriptors such as ScanContext [568] typically fail in such situations. Other approaches using object-level clusters, such as SegMap [285] and InstaLoc [1274], have been developed using higher level semantic features and can be more successful in such situations.

In summary, research needs to keep in mind these specific challenges when developing LiDAR place recognition methods.

## 8.3.1 Problem Definition

In this section, we will focus on the task of place recognition —the loop closure candidate detection problem. Given a query (i.e., a scan represented as a point cloud), the objective is to retrieve corresponding entries from a database that are similar to the query. The database (i.e., the previously visited places) is a set of disjoint place descriptors spatio-temporally acquired in an explored region.

A key consideration in LiDAR place recognition is the robustness of the retrieval method to variations in the sensor type, acquisition time, and robot pose. For example, the LiDAR type used in a query may difer from that used to create the database if diferent LiDAR devices were employed. Furthermore, a temporal gap between mapping $( i . e . ,$ building the database of visited places) and revisiting a place at a later point in time is inevitable. This temporal gap might lead to structural changes in the environment as well as diferences due to dynamics caused by moving objects or people. However, the most significant variation arises due to changes in the robot’s pose. Translation and/or rotation shifts between the database and the query result in diferent appearance of the captured sensor data of the same environment. Consequently, LiDAR place recognition methods must be robust to pose variations and environmental changes at the same time.

If a method cannot determine pose variance but still can correctly identify a candidate, it is said to have invariance. If it can also estimate pose variance, it is described as having awareness of the revisit pose variations. Researchers have particularly focused on this awareness property for two main reasons. First, it serves as a good initial guess for fine registration, which is crucial when establishing the SE(2) or SE(3) constraints required for estimating precise loop closures (see Section 8.4.1). Secondly, working towards the more complex goal of awareness can naturally enhance the invariance capability (e.g., estimating heading changes [563] or inferring the degree of overlap [188]).

## 8.3.2 Methods for LiDAR Place Recognition

In addition to achieving invariance and awareness, we should note that point cloud representations with diferent levels of granularity have been proposed to address the unstructured nature of the raw LiDAR measurements and to ensure real-time place retrieval performance for large-scale robot autonomy. Approaches for descriptorbased LiDAR place recognition can generally be categorized as using either local or global descriptors for retrieval and matching in the database. That said, there are variants that as well as using descriptor distance for place recognition also directly learn a place similarity function. In the following, we will discuss these diferent paradigms in more detail.

## 8.3.2.1 Local Descriptors

In the early days of LiDAR place recognition research, and corresponding to the evolution of visual place recognition methods (e.g., SIFT [699], ORB [953], DBoW2 [362]), computing local keypoint descriptors was a natural approach both for 2D [1096] and 3D LiDAR sensors [101]. However, 3D local descriptors specifically developed for dense RGB-D point cloud registration or object recognition [960, 961, 1103] typically struggle to be adapted to the sparsity and unstructured nature of LiDAR sensing —particularly in outdoor scenarios. To mitigate this sensitivity, methods have been proposed that use the statistical distribution of local keypoints (e.g., histograms) [462]. However, these descriptors remain limited to a local neighborhood, which results in poor descriptiveness due to the lack of metric structural context from across an overall scan.

## 8.3.2.2 Global Descriptors

In contrast to the local approaches, global descriptors leverage the higher level patterns in the entire scan rather than concentrating on low-level local keypoints. These methods aim to address the lack of structure by building a simpler and coarser representation. In turn, this often results in matching methods which are more computationally eficient. Two coarse representations which have been widely used to generate global descriptors are as follows:

Bird’s-eye-view (BEV). BEV representations transform a 3D point cloud into a structured, coarse-grained, top-down image using either a polar representation or a sparse grid representation. Scan Context++ [563, 568] and RING++ [703, 1212] are examples of approaches using this representation.<sup>3</sup> The former proposed a yaw alignment matching algorithm to achieve orientation invariance, and the latter theoretically proved its invariance and awareness by leveraging the Radon transform.

Range images. As an alternatively to using a 3D point cloud, a range image (see Section 5.1) provides a structured, well-aligned representation of a scan. The recent advancements of dense multi-beam LiDAR technology (see Section 8.1), has made representing LiDAR data as a dense range image a much more suitable approach. The advantage here is that approaches can directly borrow well-established tools from the computer vision field such as convolutional neural networks (CNN) [608] or Vision Transformers [280] to extract a detailed feature representation. Overlap-Net [188, 720] showed that yaw invariance can be achieved by applying an overlap loss to the range image, while more recently FRAME [1028] demonstrated LiDAR place recognition in mines using range images.

## 8.3.2.3 High-level or Combined Descriptors

To derive a descriptor for a scan (either local or global), some approaches use a hybrid strategy or even learn directly a similarity function for place recognition.

Segmentation-based approaches. As discussed previously, representing a scan with a single descriptor can be vulnerable to structural aliasing. Because of this attempts which describe a place using a set of meaningful objects or segments have been proposed to increase uniqueness and descriptiveness and to avoid perceptual aliasing. These methods include SegMap [285] and InstaLoc [1274].

Descriptors which bridge between local and global. The previously introduced global descriptors are typically efective only when the point cloud projections are consistently aligned in a specific direction (i.e., top-view or spherical-view). This requirement may restrict the application of the method to vehicles traveling with predictable directions along roads. To address this issue, BTC [1258, 1259] was proposed that utilizes both local and global descriptors. This approach aims to maintain the local geometry and the overall structure of a scan, efectively combining the strengths of each type of descriptor.

Direct 3D Data Processing. More recently, data-driven approaches have been proposed that can achieve retrieval using the raw unstructured points directly without handcrafted rules. In particular, deep learning-based methods [1119, 156] have been proposed to extract robust point-wise features that are resilient to the diverse sensor sparsity and local surface distributions. In particular, Cattaneo et al. [156] showed that a pipeline designed with a triplet loss for place discrimination and differentiable relative pose estimation can achieved improved registration and overall benefits LiDAR SLAM. This can also be interpreted as a evidence that focusing on awareness can enhance both invariance and discriminative capabilities.

## 8.3.3 Summary of LiDAR Place Recognition

In summary, LiDAR place recognition has the same role as visual place recognition and shares common attributes and its performance is defined by the same metrics.

The sensor modalities and related techniques are often complementary as in many of the locations where LiDAR place recognition fails, visual place recognition succeeds; and vice versa. In a mobile robot navigation system, both sensor modalities are often used to ensure robust and reliable results.

In the next section, we will describe how LiDAR place recognition can be used with pose-graph optimization to correct the unavoidable drift which occurs with LiDAR odometry so as to form a consistent, accurate, and scalable LiDAR SLAM system.

## 8.4 LiDAR SLAM

The purpose of the LiDAR odometry systems described in Section 8.2 is to estimate a locally consistent motion of the sensor as it moves through the environment. However, this motion estimate will inevitably accumulate drift as the device travels a long distance.

To counteract this drift, a LiDAR SLAM system can maintain a globally consistent estimate for the entire history of the mapping operation by recognizing when the sensor returns to previously visited parts of the environment. These recognition events are known as loop closures and they can be used by the SLAM system to correct not just its current pose estimate but also to revise the full trajectory of previous pose estimates —as well as the corresponding map representation.

A key property that we seek for a LiDAR SLAM system is that it maintains a globally consistent map. This requires the system to achieve consistency between our current observations and past observations in a single map representation. This task is particularly challenging in large-scale and potentially dynamic environments.

![](images/3955997a8f2793872abf36dff448ecc0d7b4c76c3fc5df84ae692f0059969994.jpg)  
Figure 8.7 Conceptual structure of a typical LiDAR SLAM system composed of multiple components: (1) Odometry estimates the robot/sensor pose, (2) Loop Closure Detection determines if a place has been revisited, (3) Pose-graph optimization uses the loop closure constraints to correct the pose trajectory using factor graph optimization, and (4) Map Update uses the most recent pose trajectory to revise the map representation.

In Section 8.2, we discussed the development of approaches for LiDAR odometry which can achieve drift rates as low as one meter per kilometer traveled and then in Section 8.3 we reviewed diferent methods which carry out LiDAR place recognition to determine loop closures. LiDAR SLAM encompasses the techniques necessary to bring these components together to maintain a consistent trajectory and map representation —and to do so in real-time while running on-board a robot, vehicle or sensing system.

Most contemporary LiDAR SLAM systems [66, 846, 282, 265, 1253] are composed of the components shown in the system diagram in Figure 8.7. In the following Section 8.4.1, we will discuss this structure in more detail. We will then focus on the key steps of backend optimization and map update in Section 8.4.2 and describe some common techniques for integrating loop closures to correct the robot trajectory during backend optimization.

Advanced topics for LiDAR SLAM include multi-session and multi-robot mapping. These topics focus on how to fuse multiple mapping sessions into a common global reference frame —which can be either concurrent (running live on multiple robots and solved in real-time) or long-term (aligning multiple maps over time so as to infer environmental change). Multi-session and multi-robot SLAM will be covered in Section 8.4.3, where we will also provide an overview of common solutions and challenges.

Finally, as LiDAR SLAM has matured, it has brought into focus other advanced topics. Safety critical systems such as autonomous vehicles rely on SLAM so we have to consider the robustness and the scalability of LiDAR SLAM system. These topics will be discussed in the final part of this chapter in Section 8.5.

## 8.4.1 Structure of a LiDAR SLAM System

When implementing a LiDAR SLAM system [66, 846, 282, 265, 1253] it is common to decompose it into several modules, one module maintains a relative pose estimate using an odometry estimator, and other modules identify and use loop closures to reestimate the pose trajectory and to then update the associated map representation to account for this revised pose trajectory. A LiDAR SLAM system usually consists of an odometry estimation module that runs at the sensor frame rate of the LiDAR sensors (e.g., 10 Hz) with the other modules operating at a lower rate (e.g., 1 Hz).

More concretely, consider Figure 8.7. Here, (1) an odometry component estimates a pose in a frame-to-map fashion using the currently active local map. For the odometry module, the most common approach is to register the incoming laser scan to a rolling/active local map (as discussed in Section 8.2). This odometry module will typically only have access to data from the direct vicinity of the sensor —often called an active local map.

Next, (2) a LiDAR-based place recognition method identifies potential loop closure candidates as discussed in Section 8.3. Place recognition only identifies that two places (or more specifically observations taken at those two places) are similar. To determine a precise relative transformation estimate between those two places requires fine registration of the corresponding LiDAR scans (typically using ICP).

To initialize the registration, a suficiently good initial guess of the relative transformation is needed. Geometric priors (taken from the existing pose-graph) can be used for small pose-graphs. Where no geometric prior can be used, modern global registration methods which do not rely on an initial guess but can robustly estimate a relative transformation have been developed [1223, 665]. These methods work well in situations with low overlap between the pair of scans.

Heuristics, such as the travel distance or time diference between two loop closure candidates or the degree of confidence in a RANSAC-based alignment for geometric verification, can be used to determine the validity of a loop closure candidate and to avoid adding false loop closures to the pose-graph.

Module (3) is the backend PGO step. It uses the full set of SLAM constraints to solve for an optimized pose-graph and to update the pose trajectory. We will discuss PGO in more detail in the following section.

Finally, in (4) a map update mechanism integrates the sensor measurements into a combined map representation according to this corrected trajectory. There are several potential approaches for this. The most common approach is to use the points directly [256] while approaches such as surfels [282, 846, 66] and implicit representations [1253, 265] attempt to improve the quality of the underlying map or seek to achieve a stronger probabilistic foundation. We refer the reader to Chapter 5 for technical details and discussion about the diferent dense map representations.

In the next section, we will discuss backend pose-graph optimization in more detail and how the full map representation is typically updated.

![](images/b3a888088d32286206a9cee8c0ef23cfd3c0935eec1445c57326036ed908ae30.jpg)  
Figure 8.8 A SLAM problem represented as a pose-graph. Each node represents the pose of the sensor whereas the edges represent the constraints coming from odometry (orange) and loop closures (magenta). A Prior Factor fixes the graph origin. Optional Attitude Factors can be used to constrain the pitch and roll when inertial sensing is available. From [887].

## 8.4.2 Pose-graph Optimization and Map Update

The key part of a LiDAR SLAM system is updating the pose trajectory and map representation after a loop closure has been proposed and verified. As the local pose estimate will contain drift, the error in the local pose estimate needs to be accounted for in an updated pose trajectory. Additionally the existing map representation, integrating the past measurements, will also need to be updated.

Pose-graph optimization (PGO) is typically used in the SLAM backend. PGO corrects the estimated trajectory to respect both the odometry constraints and loop closure constraints identified when revisiting already observed places. As introduced and discussed in Part I, see Chapter 1, a factor graph can be used to represent these constraints (as shown in Figure 8.8). Because the graph is made up of only relative pose constraints, it is commonly referred to as a pose-graph. An example of a point cloud map before and after loop closure detection and pose-graph optimization is shown in Figure 8.9.

The constraint set can be optimized using general-purpose solvers such as g2o [614] and GTSAM [253]. To achieve real-time performance, with a pose-graph of increasing size, it is necessary to iteratively resolve a continually growing optimization problem. However, the constraint set is typically sparse —with few interconnected edges. Sparse matrix factorization methods which reorder and relinearize the underlying system of equations allow graphs of over 1000 nodes to be updated in a fraction of a second. For further reading, refer to Chapter 1, iSAM2 [534], and HOG-Man [401].

Note that while 1000 nodes corresponds to a large pose-graph, one must consider scalability. It is common to add odometry constraints only relatively sparsely —not at sensor rate (e.g., 10 Hz) but instead every few meters traveled. Another approach is to subdivide the mapped environment into submaps of fixed physical size (say 30 m traveled) each with a corresponding pose-graph node. It is then assumed that within these submaps the odometry will be locally accurate making re-adjustment of the trajectory inside the submap unnecessary. This approach allows pose-graph SLAM to scale to city-sized maps.

![](images/8760f193ca5639fc515f2aa6794fa36de84ff53e1ba04443bf55671c5ff07218.jpg)  
Figure 8.9 By incorporating loop closure constraints, a SLAM system can create a globally consistent map of revisited locations. The left shows the odometry-only trajectory of a revisited place with visible misalignment between the original visit (blue) and the current visit (purple) to this junction. The right shows the result after pose-graph optimization when loop closures have been integrated resulting in a consistent map of the road junction.

In the backend, pose-graph optimization has to account for pose estimation errors by the odometry as well as errors in the loop closure constraints. To properly model the uncertainty in the pose estimate of the odometry poses, we can also estimate data-driven covariances to distribute the error in the pose-graph optimization sensibly [622, 159] —for example using high covariance of edge constraints where the drift rate is likely to be higher. Finally, to account for incorrect loop closure constraints and bad configurations of the pose-graph, there is a body of research into methods for robust pose-graph optimization [12, 153, 1059, 1060, 1221] which can down-weight, disable or ignore pose-graph constraints which would otherwise cause the map to degrade or diverge (see Chapter 3 for an extensive discussion).

After pose-graph optimization, the full robot trajectory will now be globally consistent, but the efect of this update also needs to be reflected in the map itself. For this purpose, a common approach is simply to re-build the map using the past observations. This would require a system to store the previous observations indefinitely —which can quickly become unsuitable in large-scale environments. An alternative approach is to deform the map representation [846] or to directly link map elements (i.e., such as surfels or submaps) to poses, to allow map deformation in a more scalable manner.

![](images/9486564fa39e44f6280fc336f2f98ed9d0420b72fcf33f92aaa61db3e95560f7.jpg)  
Figure 8.10 Comparison between (a) naive direct alignment of two global point clouds and (b) multi-mission pose-graph relaxation. (a) Point-to-point distances between the two global point clouds shows “double walling” causing phantom change to be hallucinated. (b) Multi-mission relaxation reduces point-to-point distances with structures being more clearly reconstructed. From [951] (©2024 IEEE).

## 8.4.3 Multi-robot and Multi-session LiDAR SLAM

With the development of mature single-robot single-session LiDAR SLAM systems, there is interest in extending these systems to support multi-robot and multi-session applications. This would be useful because it would allow incomplete maps to be extended into newly scanned territory or for multiple field robots to coordinate their activities using a common map representation. Another use is to co-register maps taken in the same area over time to infer longitudinal environmental change, e.g., for security or monitoring applications.

One initial point which is necessary to make is that while modern LiDAR SLAM systems are increasingly accurate —with one meter drift per kilometer being typical in open space— there will always remain some small error within a SLAM map. Simply taking the final point cloud map from two individual SLAM missions and co-registering them will result in locations where point cloud alignment is inconsistent as shown in Figure 8.10.

Initial work in this space focused on how to carry out joint backend optimization of multiple mapping sessions. One approach is to simply transfer the individual constraints from the set of SLAM instances into a single global map representation. An alternative approach is to build each map individually —each with their own coordinate frame, nodes and edges. To link them to one to another, Kim et al. [562] introduced an auxiliary variable called an anchor node which accounts for the diferent map coordinates of the individual SLAM missions. This node allows easy global alignment of each individual map and has been used for both multi-session visual SLAM and LiDAR SLAM [749, 565].

![](images/a20aaec77d6d3fd84ded54dcde4615c9b3af73e2a7a4972466a4c83f7a1a780d.jpg)  
Figure 8.11 A multi-session SLAM map of a construction site. Five diferent mapping sessions are merged together by establishing inter-session loop closure constraints (in red) and adjusting a joint optimization of the five mapping session trajectories (in green).

As described in [284], aside from the backend optimization, one must determine how loop closure constraints can be established between entirely disconnected SLAM missions. Unlike in the single session SLAM case, there is initially no geometric prior to form the first inter-mission constraint —with multi-session SLAM relying entirely on place recognition to relate maps to one another.

## 8.4.3.1 Multi-robot SLAM

Real-time multi-robot SLAM goes one step further, solving the multi-session mapping problem, but doing so with data collected in real-time by robots operating in the field. Estimating a combined map from multiple platforms in real-time allows a robot team to coordinate mission planning, optimally select frontiers of exploration, and to avoid wasted efort returning to a location mapped by another robotic team member. Achieving this capability can allow a team of robots to operate in concert —eficiently exploring territory, identifying which routes are free of obstacles, and perhaps identifying people or objects of interest. This capability is relevant for search and rescue as well as military applications.

One can distinguish between systems which are centralized or decentralized. Centralized systems may transmit sensor measurements to a base station and then compute a combined map at that location. This may be so that the field robot’s compute and sensing is kept as simple as possible, for example the mobile robots used by Amazon and Ocado for warehouse operations. On the other hand, decentralized (or distributed) systems would instead build a SLAM map on each individual robot with merging of the set of robot maps being achieved at a base station.

![](images/4c1802ace8a87abd48d81909f0691f6d44c8d38ce3922f3a743c6a4a7c03adaf.jpg)  
Figure 8.12 Multi-robot SLAM progressed from 2D to full 3D between the 2010 MAGIC challenge to the 2021 DARPA SubT Challenge. The pictures show the winning University of Michigan and Cerberus Teams from the two challenges. Image courtesy of Edwin Olson and Cerberus team.

To describe the evolution of the state of the art we refer to two major international multi-robot exploration challenges. The first one is the Multi Autonomous Groundrobotic International Challenge (MAGIC), which was held in Brisbane, Australia in 2010. This challenge involved teams of wheeled robots executing a reconnaissance mission in a 500 m 500 m challenge area to correctly locate and classify simulated threats. The winning team, Team Michigan, fielded 14 3D-printed robots equipped with 2D LiDAR scanners [828] as well as cameras (to identify loop closures). Each robot carried out 2D LiDAR odometry and transmitted its pose-graph constraints to a base station which assembled a global 2D multi-robot map.

Research progress over the last decade was evidenced by the DARPA Subterranean Challenge [600] which was held in Louisville, Kentucky in 2021. It posed a similar challenge to competing teams at MAGIC —to explore unknown environments— but with the robots operating in more complex 3D underground environments with stairs, curbs, and ramps. Here 3D multi-beam LiDAR was heavily used but also augmented with visual odometry, wheel/legged and, in some cases, thermal odometry to overcome degenerate circumstances where LiDAR odometry can fail such as in narrow environments in the underground tunnels.

The SubT teams published an overview article which provides a comparison between the fielded systems [299]. Each team used a semi-decentralized approach with individual robots building pose-graph-based SLAM maps on board with a multirobot SLAM map created at a single central base station. Key challenges included compression and communication as the robot teams needed to maintain a dynamic wireless mesh network to transmit data back to this base station. For example, the WildCat SLAM system from CSIRO [601] was notable for using a compressed surfel representation to represent local submaps. These surfel maps took up much less space than the raw point clouds —greatly reducing the bandwidth needed to transmit the map and pose-graph constraints to the base station computer. During the finals, a complete map took only 21.5 MB per robot.

As mentioned above, the most complex problem is fully distributed SLAM system where each robot platform is tasked with building and maintaining a representation of the overall combined map subject to communication and scaling constraints. Some existing approaches [493, 1095] have explored how to do this and focused on the mechanisms to share the set of constraints and local submaps progressively with each robot. Issues of consistency are key in this topic.

## 8.5 Further Readings & Recent Trends

LiDAR SLAM has seen significant advancements over the last decades —especially since the introduction of LOAM [1269]. Improved odometry with high accuracy [1211, 999, 1133, 256] and eficient pose-graph SLAM systems [66, 265, 846, 903] have also been developed. However, despite this progress, there are still unsolved problems and challenges to tackle.

Robust and Resilient Perception. A recent robustness evaluation by Zhao et al. [1288] identified that LiDAR SLAM systems struggle to perform efectively in cluttered and unstructured environments. Structure-less corridors, underground mines and extreme weather conditions such as snow, fog, and dust are other challenging situations pointed out in a review by the DARPA Subterranean Challenge competitors [299].

Furthermore, the performance of current LiDAR SLAM systems is typically demonstrated experimentally and lack formal robustness evaluation. Best performance is achieved through feature engineering and manual parameter tuning, which perhaps ought to be dynamically adjusted according to the operational scenario as in KISS-ICP [1133]. Future improvements in this regard should consider actively adapting algorithm behavior to account for changes in the environment through introspection.

With regard to place recognition, there are a broad spectrum of research directions. Key survey papers such as [1007, 1249] ofer a comprehensive foundation on the topic. Ongoing research includes methods for robust retrieval which generalize across the various categories of LiDAR sensors [529]. Other research looks to achieve heterogeneous place recognition between LiDAR and other modalities such as radar [1248] and OpenStreetMap [211]. Researchers have also successfully leveraged the LiDAR’s intensity information [1000, 1151], alongside traditional XYZ data to improve performance. Long-term place recognition across multiple mapping sessions [566] is another promising research topic; as is change detection and lifelong map management [565, 1251].

Multi-Sensor Fusion. Fusing multiple sensors with complementary characteristics is a key route to more robust and resilient robotic systems. Degraded perception of a particular sensor can be ameliorated by fusing a diferent and complementary sensor, e.g., radar works well in rain or smoke; or using visual feature tracking in a tunnel where LiDAR fails [1191, 1287]. However, when integrating additional sensors with LiDAR, we inevitably acquire a plethora of sensor data, leading to redundancy. There are open questions about how to achieve a balance between redundancy and lightweight computation. Furthermore, one must consider how to eficiently select the most reliable information when fusing estimates of multiple sensors. Solutions range from early fusion approaches (using a single tightly coupled estimator) and late fusion approaches (where separate individual-sensor pose estimators are combined).

Finally, another practical consideration is that multi-sensor systems may lack accurate calibration and individual sensors may not be precisely synchronized. This places a burden on the underlying estimation system making full, tight sensor fusion dificult to practically use over extended time intervals. There is still space for research into these intriguing research questions.

Uncertainty and Bayesian Estimation. Closely related to the question of how sensors can be reliably fused is the question of uncertainty estimation in LiDAR SLAM. Properly calibrated measures of sensor uncertainty are required to probabilistically fuse multiple pose estimates. However, most successful LiDAR-based approaches rely on ICP, which does provide a calibrated and robust estimate of the pose uncertainty.

While some early approaches [622, 159] for approximating covariance exist, the estimated uncertainty used in LiDAR SLAM system is often unreliable. As a result, algorithms often use fixed odometry covariances during backend pose-graph optimization. A more introspective handling of uncertainties in the odometry process has the potential to address degraded pose estimates at an earlier stage. There are still many open questions regarding uncertainty estimation, where robust solutions would support the development of more resilient SLAM systems as well as multi-modal SLAM.

Deployment in Closed-Loop Autonomous Systems. A final consideration is how LiDAR SLAM performs in a desired final application. These applications are as varied as light-weight aerial vehicles flying through forests, handheld devices scanning construction sites, and self-driving cars operating in poor weather. The traditional electronics requirements of Size, Weight, and Power (SWaP) are augmented with additional parameters of accuracy, robustness, computation, and latency. For instance, the ability for a self-driving car to respond to a potential upcoming collision with as little delay as possible is afected by the computational latency of the LiDAR system. Thus in certain applications the most accurate and computationally complex system may not always be the preferred solution.

Martin Magnusson, Christofer Heckman, Henrik Andreasson, Ayoung Kim, Timothy Barfoot, Michael Kaess, and Paul Newman

In this chapter, we explore the use of radar (RAdio Detection and Ranging) in SLAM. Compared to cameras and lidar, radar is somewhat undersubscribed. However, due to its ability to work in poor visibility, at long range, and to natively produce velocity information, its popularity is on the rise. We begin by discussing the types of radar sensor typically used in robotics, their unique sensing principles, and some of the challenges that come along with radar (Chapter 9.1). We then discuss radar filtering, radar odometry (Chapter 9.2), place recognition (Chapter 9.3), and finally SLAM (Chapter 9.4); Figure 9.1 shows how these pieces fit together. We conclude with a discussion of radar SLAM datasets (Chapter 9.5) and an outlook on the use of radar moving forward (Chapter 9.6).

![](images/6485c2c26e82c18f65c65a46aae600bc8df605abf181ff0ef5eb02e52a4bf1d1.jpg)  
Figure 9.1 The flow of information in radar-based SLAM follows the same pattern as other SLAM systems with the details of each process being slightly altered to suit the specifics of radar.

## 9.1 Introduction to Radar

## 9.1.1 Sensor Types

In the following section, we introduce two of the most common radar types that are encountered in robotics: spinning radar and system-on-a-chip (SoC) radar (see

![](images/63ca43f7732c9806485524adeb61d94b9f31e3230945fe3cf6877774edbcb98e.jpg)  
Figure 9.2 The two main categories of radars used in robotics are spinning (top left) and phased-array SoC (top right). Some examples of each are shown along with the main data product produced by each type of sensor. Often these raw data products are subsequently turned into a sparse point cloud using radar filtering (bottom). The bottom right part shows three point clouds from a tunnel with smoke. Red: LiDAR point cloud with much limited range due to the smoke. Blue: point cloud from 2D spinning radar (tunnel walls in all directions are clearly visible also at a distance). Green: 3D point cloud from SoC radar (walls and ground surface in front of the vehicle are visible). The bottom left part shows thermal and visual/RGB image data from the same scene, and the polar radargram from which the blue point cloud is extracted.

Figure 9.2). Each has its strengths and drawbacks. They difer mainly in how they are ‘actuated’ with spinning radar mechanically rotating a single antenna and SoC using multiple antennas whose signals are combined to deduce the angles and ranges of objects reflecting the transmitted signals.

## 9.1.1.1 Spinning Radar

Utilizing a rotating radar sensor, a spinning radar —sometimes referred to as scanning radar or imaging radar— crafts precise polar representations of its surroundings, exemplified in Figure 9.2. The main data product produced is a polar radargram. Imaging radars are distinguished by their ability to detect objects at distances exceeding 100 meters. In some modes, the velocity of those objects can also be determined by exploiting the Doppler efect.

In contrast to LiDAR systems, spinning radars are limited to providing data on a two-dimensional plane, lacking the capacity to measure the elevation of detected objects. This limitation persists even though reflections might originate from various elevations within the antenna’s vertical beamwidth. Additionally, because their operational mechanism involves transmitting and receiving a single pulse for each antenna angle, these radars sometimes do not ofer velocity data, which requires multiple pulses for calculation. More recent work uses the signal from multiple neighbouring azimuths to recover velocity.

Constructing a mechanically spinning 3D radar with multiple vertical beams, similar to currently widespread 3D LiDAR sensors, is not practically feasible due to the much larger size of the antennas and focusing mechanisms compared to laser diodes.

## 9.1.1.2 SoC Radar

System-on-a-chip (SoC) radars integrate processing units within a minimal set of chips, which are either directly mounted on patch antennas (arrays of transmitters / receivers) or incorporated into the printed circuit board itself. SoC radars are characterized by their lightweight design and reduced power requirements compared to spinning radars, thanks to their integrated architecture and lack of moving parts. The performance in terms of accuracy and resolution for SoC radars hinges on the design of the antenna array and the proprietary processing techniques manufacturers employ to integrate measurements from multiple antennas. The primary output produced is a radar datacube.

SoC returns are typically mapped in spherical coordinates by azimuth, elevation, and range. Given that radial velocity adds another dimension, these radars are often referred to as 3+1D or 4D. Conversely, systems with limited or absent vertical resolution, due to a scarcity of vertically aligned antennas, are denoted as 2D array radar systems, producing 2+1D data products.

## 9.1.2 Radar Sensing Principles

In this section, we will discuss basic operations, antennas and datatypes, and challenges as they relate to robotics applications. We outline each in the following sections and designate how radar diferentiates itself from other rangefinding sensors.

MmWave (millimeter-wave) radar is designated by radar systems whose electromagnetic wavelengths are between 1–10 mm with frequency ranging from 30– 300 GHz. Within this range, most radar sensors operate in the 76–81 GHz segment of the spectrum due to automotive applications such as ADAS systems having spectrum carve-outs in this range.

## 9.1.2.1 Radar Cross Section

The process of mmWave radar involves emitting electromagnetic pulses that travel until they meet objects, bouncing back towards the radar. The Radar Cross Section (RCS), influenced by an object’s material composition, size, and shape, plays a crucial role in determining how strongly each object reflects these electromagnetic pulses. Essentially, the RCS represents the size of a hypothetical sphere that would reflect the same amount of energy as the target object. Thus, larger and more solid structures such as vehicles and thick concrete walls exhibit a higher RCS compared to smaller objects or pedestrians, which present a lower RCS.

The term radar intensity refers to the strength of the radar’s echo from an object, which is a function of the radar’s transmitted power and the object’s RCS. In essence, this intensity is chiefly determined by the radar’s emitted power and the RCS of the encountered target. Literature highlights that this measure of intensity has been crucial for extracting semantic details about objects or aiding in navigation, as stronger echoes tend to be associated with distinctive and easily recognizable features in the environment [1240]. The strength of the radar signal is influenced by several additional factors, including the type of antenna used, the characteristics of the electromagnetic pulse emitted, and the antenna’s ability to detect returns from objects.

## 9.1.2.2 FMCW Ranging

A radar comprises at least one transmitting antenna (TX) and one receiving antenna (RX).<sup>1</sup> In the context of FMCW radar, the TX antenna’s role is to emit an RF pulse that steadily increases in frequency, known as a chirp. This is often called sawtooth modulation; we will also discuss triangular modulation, another chirp-based technique, later on. These chirps bounce of objects in the environment, and the RX antenna captures the echoes (see Figure 9.3). Upon receiving a chirp, the signal is amplified and then mixed with, or subtracted from, the original TX chirp to generate an Intermediate Frequency (IF), which is also a signal. The mixing process results in an IF that is a sine wave of constant frequency $f _ { 0 } ,$ due to the identical slope of both the transmitted and received signals. The performance of the radar system, including its range and velocity detection capabilities, is afected by various chirp parameters such as the bandwidth (the diference between the initial and final frequencies), the chirp slope, and the duration between chirps.

![](images/ab8cdc8bfb238ce9142f4dad7c7d43d18c7a047bed5573bff0b95eac5e6f28cf.jpg)  
<sup>�</sup> AngleFigure 9.3 A frequency modulated continuous-wave (FMCW) radar works by emitting �chirps (waves of increasing or decreasing frequency) that are reflected of targets and then <sup>Tx</sup>received. Range (and velocity) are determined by analyzing the frequency (and phase) shifts of the reflected signal compared to the transmitted one. Two common frequency Rx <sup>�</sup>modulation strategies are sawtooth and triangular; the advantage of the latter being that �the Doppler frequency shift can be disentangled from the range shift.

The main idea of this frequency modulation is to encode temporal information onto a continuous wave, enabling the execution of range calculations. This technique stands in contrast to amplitude modulation, where the precision of the returned signal depends exclusively on the frequency’s bandwidth. This characteristic renders frequency modulation more resilient to issues such as signal-to-noise ratios and the RCS of targets, which might otherwise afect the clarity of individual returns. In the context of FMCW radar, where waves are interspersed with unique intervals between pulses in a train, the signals maintain coherence. This coherence allows for each chirp to be accurately associated with its originating transmit-receive pair and its position within the sequence, facilitating the correlation of signals in terms of both amplitude and phase. This represents a significant evolution from the older time-of-flight pulsed radar systems, which relied on incoherent amplitude measurements and required the expertise of skilled operators to sift through clutter and isolate significant signals.

In FMCW, the calculation of distance relies on the temporal gap between when a signal is sent and when its echo is received. Utilizing the speed of light, $c ,$ and the initial arrival time, $t _ { 0 } ,$ the distance, $d ,$ to an object is computed as

$$
d = \frac {c}{2} t _ {0}.\tag{9.1}
$$

The top-left portion of Figure 9.3 shows the region where an IF signal is generated using green dashed lines. The diference between the transmitted and received signal is an IF signal. The IF signal is a sine wave si $\mathsf { \Omega } _ { \mathsf { l } } ( 2 \pi f _ { 0 } t + \phi _ { 0 } )$ whose frequency is proportional to a constant $f _ { 0 }$ that spans from $t _ { 0 }$ to $T$ that only depends on the distance to the target, and is ofset in phase by $\phi _ { 0 }$ .

The frequency, $f _ { 0 } ,$ is defined as a function of the distance to the target, the duration of the transmitted chirp, T, and the bandwidth of the transmitted signal, $B = f _ { M } - f _ { m }$ . The bandwidth and transmit time are related to the slope of the chirp, $S = B / T$ , as

$$
f _ {0} = \frac {2 B d}{T c} = \frac {2 S d}{c} \Rightarrow d = \frac {c f _ {0}}{2 S},\tag{9.2}
$$

where we now have the distance, $d ,$ as a function of the (measured) intermediate frequency, $f _ { 0 }$

In this section, although the equations are presented within the frequency domain, the real-world capture of signals predominantly occurs through digital sampling. This sampling is done at a high rate, typically every 100 nanoseconds or less, using a high-frequency Analog-to-Digital Converter (ADC). Following this, the sampled data undergoes a sequence of Fast Fourier Transform (FFT) operations. These operations generate graphs depicting frequency and amplitude, from which signal peaks can be discerned. The identification of range frequencies is achieved by applying FFT to the data from a single chirp and its corresponding return signal. To calculate velocity frequencies, a series of FFTs are executed on multiple chirp and return sequences.

## 9.1.2.3 Determining Distance and Velocity with Sawtooth Modulation

Sawtooth frequency modulation (see Figure 9.3) is typically used with SoC radars and some spinning radars. The phase of the IF signal, ϕ , can be expressed as a function of the wavelength λ of the signal and the distance d:

$$
\phi_ {0} = 2 \pi f _ {m} t _ {0} = \frac {4 \pi d}{\lambda}.\tag{9.3}
$$

While both the base frequency $f _ { 0 }$ and phase $\phi _ { 0 }$ are functions of distance $d ,$ the phase is only valid for suficiently small distance values and is subject to anglewrapping. Thus this is typically not used for range estimation but to measure small changes $\Delta d$ where the phase responds linearly in velocity estimation.

For radial velocity estimation (see Figure 9.4), at least two sequential up chirps are employed as shown in the top left of Figure 9.3. As the distance in time between each chirp in the sequence is small, on the order of 40 microseconds, the range measurement from both samples and thus the relative IF $f _ { i }$ and $f _ { i + 1 }$ are nearly identical. However, the IF signals will possess distinct phases. This phase disparity $\Delta \phi$ corresponds to a motion of the object. The estimated velocity v is determined

![](images/a43c559321a1564a744d1f4cac675b0a1abf0ca83c92f632fbc1a8641dbdc68d.jpg)  
Figure 9.4 Radar is able to use the Doppler efect to measure the radial velocity between a unit and the scene it is imaging.

from the phase diference,

$$
\Delta \phi = \frac {4 \pi \Delta d}{\lambda} = \frac {4 \pi v T}{\lambda},\tag{9.4}
$$

simplified to

$$
v = \frac {\lambda \Delta \phi}{4 \pi T}.\tag{9.5}
$$

As velocity is a function of phase, we note the maximum detectable velocity is unambiguous for $| \Delta \phi | < \pi$ . Thus $v _ { \mathrm { m a x } } = \lambda / ( 4 T )$ is a function of the wavelength of the signal and the time between chirps.

## 9.1.2.4 Determining Distance and Velocity with Triangular Modulation

Triangular frequency modulation (see Figure 9.3) can also be used; for example, it is sometimes used with spinning radars. In reality, when an intermediate frequency is derived from an up chirp, it comprises two components: (i) frequency shift due to the time of flight of the signal (discussed already), $f _ { 0 , t } { } ,$ and (ii) the apparent frequency shift due to the Doppler efect if there is a relative velocity between the radar and the target, $f _ { 0 , d } \colon$

$$
f _ {0, \mathrm{up}} = f _ {0, t} + f _ {0, d}.\tag{9.6}
$$

However, if we follow an up chirp with a down chirp that reflects of the same target, the sign of the temporal frequency shift will flip while that of the Doppler shift will not:

$$
f _ {0, \mathrm{down}} = - f _ {0, t} + f _ {0, d}.\tag{9.7}
$$

From these two equations we can solve for $f _ { 0 , t }$ and $f _ { 0 , d } \colon$

$$
f _ {0, t} = \frac {f _ {0 , \mathrm{up}} - f _ {0 , \mathrm{down}}}{2}, \quad f _ {0, d} = \frac {f _ {0 , \mathrm{up}} + f _ {0 , \mathrm{down}}}{2}.\tag{9.8}
$$

![](images/94ee3c464fa3e6a26cf02955d1a3bcc364b0f89d85a8af88d806782a2a60bb64.jpg)  
Figure 9.5 Angle of arrival estimation with phased-array radar. The measured diference in phase of the signal emitted by the TX transmitter antenna as received by two RX receiver antennas at a distance ℓ corresponds to the angle θ to the target.

Finally, from these two components, we can calculate the range and velocity according to

$$
d = \frac {c f _ {0 , t}}{2 S}, v = \frac {c f _ {0 , d}}{2 S T}.\tag{9.9}
$$

Notably, the range calculation presented earlier in (9.2) is not corrected for the Doppler efect whereas this one is. The downside of using triangular modulation is an increase in latency since we now require slightly older data in the calculation of range and velocity. However, we do not need to work with the phase of the signal.

## 9.1.2.5 Determining Angle for SoC radar

For spinning radar, determining the angle to a target is trivial since the beam is focused in a single azimuth direction at each time. Angle estimation for SoC radar is slightly more complex but can be achieved with a similar phase diference calculation as above.

Given multiple receiver units separated by an interval $\ell ,$ the distance disparity ∆d emerges in reflections. The angle of arrival θ can be derived from the modification of (9.3) with the geometric relation $\Delta d = \ell \sin \theta$ . The received signal must travel an extra distance ℓ sin θ to reach the second receiver antenna, as illustrated in Figure 9.5. This corresponds to a phase diference of $\Delta \phi = ( 2 \pi / \lambda ) \ell \sin ( \theta )$ between the signals received at the two RX antennas. Given the measured phase diference $\Delta \phi .$ the angle of arrival θ can be computed as

$$
\theta = \arcsin {\frac {\lambda \Delta \phi}{2 \pi \ell}}.\tag{9.10}
$$

While one TX and two RX antennas are suficient in principle for determining the angle to a target, having more than two RX antennas enables higher resolution and thus the ability to distinguish multiple nearby targets. The phase of the returned signal will be ofset by an additional $\Delta \phi$ at each RX. Sampling the signal across the RX antennas and performing an FFT on this signal sequence can be used to reliably estimate $\Delta \phi$

Radar SLAM  
![](images/2536714fdcbb28d955c776ed219adb1c6aded4e3f3f7bb16847fa9fe86f0fdd0.jpg)  
Figure 9.6 A polar radargram in (a) transformed to Euclidean coordinates in (b), where we can observe several types of radar noise that are unique compared to other sensors from a zoomed view in (c). Speckle noise returns are the most common, with ambiguous clutter circled in green. Multipath reflections develop where returns bounce of nearby walls or the ground before hitting the antenna, generating reflections of true targets. A series of repeated returns is circled in red. The original image is sampled from the Mulran dataset [567].

The above example illustrates a “SIMO” phased-array radar system (single input, multiple output). Most radar sensors used for SLAM applications are MIMO (multiple input, multiple output) with several TX and RX antennas. Rather than doubling the number of RX antennas, it is possible to achieve the same resolution by adding one more TX antenna, as long as the RX antennas can distinguish the signals from the multiple TX antennas. Diferent techniques can be used to ensure that the TX signals are uncorrelated (orthogonal); e.g., frequency division (where each transmitter uses a diferent frequency band), code division (where each transmitter sends a signal modulated by a unique code sequence), or time-division multiple access (TDMA) where each transmitter uses a diferent time slot. The same principle can also be applied to 2D TX-RX arrays that can measure both azimuth and elevation angles, thus producing 3+1D data.

## 9.1.3 Challenges to Radar Applications

Radar technology, like any sensor system, presents a range of challenges that necessitate careful consideration during development. These challenges include a variety of noise types that are particularly prominent in radar, such as multipath reflections, biased and sparse range readings due to the wide beam width, receiver saturation, and speckle noise. Illustrations of some of these noise phenomena are provided in Figure 9.6, which depicts a polar radargram transformed to Euclidean coordinates. We discuss some relevant radar filter techniques in Section 9.1.4.

## 9.1.3.1 Speckle Noise

Noise in radar measurements come from several sources, including thermal noise, electronic flaws, and varying RCS of targets. When a radar emits an electromagnetic pulse, it captures the energy reflected back by all objects within the antenna’s field of view. The interaction of this pulse with objects scatters the radar waves, leading to constructive and destructive interference. Such interactions can either produce false signals or cancel out legitimate returns received by the antenna, irrespective of the signal’s origin. These factors contribute to signal variations across the frequency domain, where the most prominent peaks represent a mix of genuine targets and false alarms. In the absence of a mechanism to distinguish between genuine and false returns, the sensor ends up generating a pattern of scattered points, commonly referred to as speckle noise. For accurate identification of landmarks, crucial for pose estimation and feature matching, it becomes essential to estimate the uncertainty around these reflections, possibly over several scans.

## 9.1.3.2 Multipath

Beyond speckle noise, multipath returns constitute another form of measurement errors, originating from varied detection paths associated with a single object. Imagine a scenario with a landmark situated in front of the sensor. While some transmitted rays may directly reach this landmark, others might only arrive at the antenna after reflecting of the ground or bouncing of a wall. To the radar, it appears as though the landmark is located beneath the road or beyond the wall, leading to the perception of what are termed ‘ghost objects’ or static outliers. The elimination of these outliers is crucial for ensuring the reliability of point cloud mapping or localization.

## 9.1.3.3 Motion-Induced Distortion

Scanning sensors, including both lidar and radar, inherently exhibit motion distortion. This is particularly true for spinning radar, which constructs each polar image through a single rotation. Consequently, if the sensor is moving, the position of a single object captured at the start and end of one rotation will difer. This discrepancy becomes significant with sensors operating at low frequencies or when the vehicle moves swiftly. For instance, the Navtech CIR 304, a commonly used imaging radar, operates at a frequency of 4 $\mathrm { H z , }$ posing challenges for accurately aligning raw frames. There are also faster spinning radars like the newer Navtech RAS3 which spins at 10 Hz, similar to many lidars, and the Indurad iSDR that can spin at up to 50 Hz. Still, when the sensor is mounted on a fast-moving platform (like a car), the motion-induced distortion can be significant.

## 9.1.4 Radar Filtering

The occurrence and distribution of false targets (Section 9.1.3) can change over time and are characterized by unpredictable parameters, rendering static filtering approaches such as simple thresholding insuficient, as they may allow false alarms to pass through. In response, researchers have devised methods to dynamically estimate the distribution of false alarms, taking these challenges into account. These filtering methods typically consider the measurements along one azimuth direction, trying to estimate which range bin(s) contain true targets and which are false alarms.

The constant false alarm rate (CFAR) filter [811] is a popularly implemented method designed to sustain a specified probability of false alarms amidst dynamically changing and uneven interference. The process begins by segmenting a signal —such as the frequency domain representations obtained post-FFT of radar ADC samples— into discrete segments known as cells. These cells are then assessed using a sliding-window approach. At the core of this window lies the set of cells under test (CUT). The intensity of the CUT is evaluated against that of the adjacent cells, referred to as training cells, which precede and follow the CUT. In some implementations, guard cells may be placed between the training cells and the CUT to prevent the local influence of the CUT from afecting the training cells’ magnitude. A decision to accept or reject a CUT set is made based on whether its intensity surpasses a calculated threshold, which is derived from the comparative intensity of the surrounding training cells, guard cells excluded. Several variants of CFAR exist, the canonical version being cell-averaging CFAR (CA-CFAR), where the threshold is computed in relation to the mean power of the training cells, scaled by a threshold multiplier that is selected from the desired probability of false alarm. A common alternative is ordered-statistic CFAR (OS-CFAR) where a more robust statistic such as the median of the training cells is used instead of the mean.

Figure 9.7 illustrates how the CA-CFAR threshold adapts to a radar signal and which range bins are selected as targets vs noise, compared to two other filtering methods.

A variant of CFAR designed and tested specifically for radar odometry is BFAR (bounded false alarm rate) [25] which simply modifies the output Z computed from a CFAR detector with an afine transformation $T = a Z + b ,$ where b is a learnable parameter that scales the output to blend between the CFAR output and a fixedlevel threshold.

While CFAR and its variants are used in many radar applications, several pipelines for radar odometry and SLAM employ simpler filtering strategies. Whereas CFAR was developed for target detection (where it may be important not to miss a weak detection), when filtering radar for use in odometry the concern is rather to retain only those points that can reliably be detected over time and from diferent viewpoints. One popular technique involves selecting the k strongest returns above a static threshold along each azimuth, with k ranging from 1 and upwards. A statistical threshold may also be employed, selecting all points with an intensity higher than one standard deviation over the mean value.

Some recent approaches instead use machine learning techniques to increase the accuracy and resolution of radar output, typically using LiDAR data as ground truth for training. Cheng et al. [195] use a generative adversarial network (GAN)

![](images/4c941393053409c9e2146a4dbd1c8e8efd0dafe7ad7c4b5e5b1f770b4ab19ec1.jpg)  
Figure 9.7 Examples of radar filtering, comparing a CFAR filter with a constant power threshold and a k-strongest strategy. The power/range plot along one azimuth direction is plotted in grey. Returns from true targets appear as spikes in the plot but there are also several ambiguous peaks. CFAR produces an adaptive threshold, plotted with a black line. Detections reported by the CFAR filter are plotted in blue, and in this case includes several “false alarms”. The k strongest filter (with k = 12 in this case) is more conservative and only returns points around the main targets. Figure from Adolfsson et al. [6] (©2021 IEEE).

to generate point clouds based on range-Doppler velocity matrices. Xu et al. [1210] train a regressor and classifier, where the regressor outputs improved, higher-resolution depth readings, and the classifier provides an estimate of whether the data is out of range. These methods strive to learn models that can retain only those returns in the radargram that correspond to a surface that would be detected by a LiDAR.

## 9.2 Radar Odometry

The goal of radar odometry is to, given a set of ordered radar readings over time, estimate the egomotion of the sensor. A radar odometry approach typically involves handling an intermediate representation, such as a set of the last N radar readings or a continuously pruned local map. The focus lies on obtaining an accurate pose estimate at a local scale. Without considering explicit loop closures, the error will eventually accumulate without bounds even for good odometry methods. Methods using spinning radar may accumulate on the order of 1–2 % translational drift per 100 m.

A particular feature of many radar sensors is the per-point ‘Doppler’ velocity estimates, which can be used to estimate odometry in a correspondence-free manner, as described in Section 9.2.1. In addition to Doppler-based methods, the relative transformation between two nearby radar scans is often estimated via spatial correspondences so as to determine which parts of one scan can be found in the other scan. Given a set of such correspondences, a distance metric can be computed and optimized. Depending on the type of scanner, how to obtain these correspondences is diferent; a spinning radar often produces a raw signal that either can be used directly as described in Section 9.2.2, to extract higher-level features containing information from the raw signal (see Section 9.2.3) or to extract range points that can be used in registration, much like in LiDAR odometry (see Section 9.2.4).

![](images/0cabd6dce45ccff6946274bb059e9c0bf50fd880e018b61ab8938a6250bd2efa.jpg)  
Figure 9.8 Example of open-loop radar odometry on the Oxford Radar Robotcar dataset [127], using the CFEAR method [8] with data from a Navtech 2D spinning radar. The ground-truth trajectory plotted in blue and the odometry estimate in orange. Point targets extracted from the radargram in grey.

An indicative example of what open-loop radar odometry using a 2D scanning radar may look like is shown in Figure 9.8.

## 9.2.1 Doppler Odometry

The radial velocity obtained from Doppler measurements can be used to directly estimate the sensor’s linear velocity. In Doer and Trommer [272], Doppler information is utilized via a combination of three-point RANSAC and a least squares problem to estimate linear velocities.

However, the rotational velocity component is not directly observable from the per-point velocity measurements in the data from a single radar —unless assumptions can be made about the kinematic model for the radar system, for example, knowing where the radar is mounted with respect to the center of rotation and assuming no skidding [551, 354]. Therefore it is common to use IMU data (or more specifically, a gyroscope) for radar odometry systems that rely solely on Doppler information from a single radar. In Huang et al. [490], a consumer grade IMU combined with cascaded SoC radars achieves low drift in diverse 3D indoor spaces. Kubelka et al. [609] compared several variants of registration-based approaches for 3+1D radar with registration-free Doppler + IMU odometry [272] and found the registration-free method to produce the lowest error, not least in feature-sparse environments, with a drift as low as 0.3% over a 4.5 km trajectory reported.

![](images/f11a8bf09a459fdc3f1f348c3533a5411923c85daba70f97e5d696f569278554.jpg)  
Figure 9.9 Factor graph representation of the radar-inertial velocity estimation system. States from K previous timesteps are jointly estimated using Doppler targets and sets of IMU measurements as constraints. Figure adapted from Kramer et al. [606].

Kellner et al. [551] show how linear and rotational velocity can be estimated from 2+1D radar data when the radar is mounted on a vehicle with Ackermann steering and the mounting point of the radar sensor with respect to the center of rotation of the vehicle is known. Galeote-Luque et al. [354] extend this to the 3+1D case and estimate five degrees of freedom (linear motion in three dimension plus yaw and pitch rotation, but not roll).

Since the Doppler-based modality of odometry is rather specific to the radar methodology, we provide an example based on Kramer et al. [606].

## Example: Doppler Odometry Factor Formulation

A straightforward approach to integrating a Doppler factor from radar is in estimating the body-frame velocity of the sensor platform over a sliding window of K previous radar measurements. These velocities are interconnected through integrated accelerometer measurements from the IMU, which can form a comprehensive system for accurate velocity estimation. The system’s structure can be represented using a factor graph, where states from N previous time steps are jointly estimated using Doppler targets and sets of IMU measurements as constraints.

IMU measurements (accelerometers and gyros) are typically afected by both bias b and gravity $\mathbf { g } ^ { w }$ . Since velocity estimates derived from radar data are free from bias, accelerometer biases can be compensated for by including them in the state vector. However, compensating for the efects of gravity requires estimating the IMU’s attitude, specifically its pitch and roll, which are represented by the orientation quaternion $\mathbf { q } _ { s } ^ { w }$ . Consequently, the full state vector is expressed as ${ \bf { \delta } } _ { \bf { { \delta } } } _ { \bf { { \delta } } } =$ $\left\lceil \mathbf { v } ^ { s ^ { \top } } \quad \mathbf { q } _ { s } ^ { w ^ { \top } } \quad \mathbf { b } ^ { \top } \right\rceil ^ { \top }$

The radar-inertial ego-velocity estimation is formulated as an optimization problem, where the cost function integrates the constraints and measurements to provide an accurate estimate of the sensor platform’s velocity:

$$
J (\boldsymbol {x}) = \underbrace {\sum_ {k = 1} ^ {K} \sum_ {d \in \mathcal {D} _ {k}} w _ {d , k} e _ {d , k}} _ {\text {Doppler term}} + \underbrace {\sum_ {k = 1} ^ {K - 1} \| \mathbf {e} _ {k} \| _ {\boldsymbol {W} _ {k}} ^ {2}} _ {\text {inertial term}},\tag{9.11}
$$

where K is the number of past radar measurements for which states are estimated, $\mathcal { D } _ { k }$ is the set of targets returned from the radar measurement at time k, $e _ { d , k }$ is the Doppler velocity error, and $\mathbf { e } _ { k }$ is the IMU error. The error terms are weighted by the covariance matrix $W _ { k }$ in the case of the IMU errors and the normalized intensity of the corresponding radar target

$$
w _ {d, k} = \frac {i _ {d , k}}{\sum_ {j \in \mathcal {D} _ {k}} i _ {j , k}},\tag{9.12}
$$

in the case of the Doppler velocity measurements where $w _ { d , k }$ is the weight for target d in scan $\mathcal { D } _ { k }$ and $i _ { d , k }$ is the intensity of target $d .$

In this example, we consider radar measurements consisting of a set of targets . For each $d \in \mathcal { D }$ , we measure ${ \left[ \begin{array} { l l l l } { r } & { v } & { \theta } & { \phi } \end{array} \right] } ^ { \top }$ representing the range, Doppler (radial) velocity, azimuth, and elevation for target d. The Doppler velocity measurement v is equal to the magnitude of the projection of the relative velocity vector between the target and sensor $\mathbf { v } ^ { s }$ onto the ray between sensor origin and the target $\mathbf { r } ^ { s }$ , both in the sensor frame. This is simply the dot product of the target’s velocity in the sensor frame and the unit vector directed from the sensor to the target:

$$
\mathbf {v} ^ {s ^ {\top}} \left(\frac {\mathbf {r} ^ {s}}{\| \mathbf {r} ^ {s} \|}\right).\tag{9.13}
$$

In this approach, it is assumed that the targets in the scene are stationary and only the sensor platform is moving. In this case, each radar target can provide a constraint on our estimate of the sensor rig’s velocity in the body-frame. The velocity error for each radar target is then

$$
e _ {d, k} = v _ {d, k} - \mathbf {v} _ {k} ^ {s ^ {\top}} \left(\frac {\mathbf {r} _ {d , k} ^ {s}}{\| \mathbf {r} _ {d , k} ^ {s} \|}\right),\tag{9.14}
$$

where $v _ { d , k }$ is the measured radial velocity of target d at time $k , \mathbf { v } _ { k } ^ { s }$ is the sensor velocity at time $k ,$ and $\mathbf { r } _ { d , k } ^ { s }$ is the vector from the sensor to the target d at time k. As previously noted, radar measurements are afected by non-Gaussian noise and radar scans often contain false target data. These challenges may be addressed by using the Cauchy robust norm with the Doppler residual. Furthermore, while this example considered the radar sensor and IMU to be at the same location and orientation, in general that is not the case; [605] includes a general derivation that illuminates the coupling between the body angular rate and the radar sensor-frame velocity.

## 9.2.2 Direct Odometry

Methods that operate on raw radargrams as the ones depicted in Figure 9.6 and Figure 9.2, as opposed to points filtered from the radargrams, are referred to as direct methods.

Direct approaches make use of classical signal processing techniques such as phase correlation and the Fourier-Mellin transform [176, 848]. Given two sequential polar radargrams, their relative rotation can be found by a translational shift in the polar coordinate frame —where a vertical shift corresponds to a change in azimuth angle. Using phase correlation, the relative orientation is selected from the pixel shift that maximizes the agreement of the two polar images. Subsequently, the translation can be refined by a similarly computing the correlation between the images in a Cartesian frame (Figure 9.6). These direct correlation-based methods assume that power returns from a specific location remain stationary over time, enabling meaningful correlation. However, this assumption often fails, especially with dynamic objects or in radar data, where noise artifacts are common.

Correlation is also used in the “Masking by Moving” method of Barnes et al. [53]. Two-dimensional correlation between the current scan and rotated copies of the previous scan is computed on a regular grid of pose candidates. However, Barnes et al. address the problem of nonstationary power returns by training a convolutional neural network (CNN) to avoid including false features that are due to noise. The CNN is trained to predict a mask that keeps only those parts of the radargram that are stationary and thus more useful for correlative scan matching.

In contrast to these direct odometry methods that use all or mostly all of the radargram, the methods in the remainder of this chapter are indirect in that they first select specific features or key points and operate on those sparser points to estimate the odometry.

## 9.2.3 Feature-based Odometry

Given that radargrams from 2D spinning FMCW radars are essentially birds-eyeview images, it is natural that several works utilize image-based feature extraction and matching techniques from the computer vision community to find correspondences and estimate odometry; such as SIFT [140, 650], SURF [473], and ORB [546] features. Callmer et al. [140] match large-scale features of islands in an archipelago and Li et al. [650] extract features from a satellite radar. Compared to camera images, the noise level is higher in radar data and highly dependent on the environment. Hence, extracting descriptors that can be used for feature matching is more dificult [474]. Feature descriptors may also be more place-variant in radar compared to other sensors, making it dificult to do data association if the sensor pose is diferent. (See also Section 9.3.) FSCD and BASD [908, 983] are examples of key-point extractors and feature descriptors specifically designed for radargrams.

The techniques above involve extracting a set of salient key points (which can reliably be detected in subsequent frames) and computing a feature descriptor describing the surrounding region of the feature point. Given the extracted feature set, the correspondences are computed using feature descriptor matching, often combined with a robust estimator, such as RANSAC. Lim et al. [663] provide an example of outlier-robust, feature-based radar odometry based on a robust estimator. Given the correspondences, the spatial distance between features is minimized, often by finding the least squares solution using Singular Value Decomposition (SVD). In general, a feature-based approach is more stable towards large initial errors compared to the registration based approaches discussed in the next section, since feature descriptors can be associated robustly compared to registration methods that primarily hinge on point proximity for data association.

Going beyond hand-crafted feature descriptors as in the examples above, keypoint extraction and feature descriptors can also be generated via deep neural networks, thus allowing features to be automatically generated [52]. One drawback is that training data including radargrams along with ground truth poses from a somewhat similar environment are required.

## 9.2.4 Registration-based Odometry

There are well-developed methods for odometry estimation based on point clouds from LiDAR ranging sensors and similar techniques can be used for point clouds extracted from radargrams or radar datacubes. In this section we describe how such radar point clouds can be computed and used for odometry estimation.

For spinning radars that provide raw signal data as shown in Figure 9.6 we first need to select which points to use by filtering those signal returns that do not correspond to a relevant peak.<sup>2</sup> As discussed in Section 9.1.4, many approaches extract a set of range readings per azimuth; e.g., using CFAR (Figure 9.7, Section 9.1.4), using noise statistics to remove redundant or noisy readings [158], BFAR [25], or simply the k strongest returns per azimuth. An exception is Kellner et al. [550], using DBSCAN clustering so as to also consider neighboring azimuth angles instead of restricting the search for targets along one azimuth dimension at a time. Models trained with machine learning so as to estimate a point cloud similar to that from a LiDAR from a radargram are also commonly used in recent methods [195, 1210]. A key challenge is to extract an adequate amount of readings; too few readings discards relavant information and too many include noise [158]. For example, CFAR has been found dificult to tune in this respect [127].

Once a point cloud has been extracted using one of the techniques above, it can either be used as-is or additional information can be estimated by examining the local surrounding region, such as normals, planes and point distributions. The registration approaches used for radar data are often similar to what is done using LiDAR based scan registration approaches (see Section 8.2); however, the noise level and sparsity in radar data makes pair-wise registration much more challenging.

A common strategy in registration-based odometry methods using radar data is to register new scans to multiple previous scans —either aggregated into a submap or as a set of individual point clouds. Registering to multiple scans is a way to compensate for several of the challenges in radar data as discussed above. By including more key frames, the odometry estimate is less sensitive to sparse and noisy radar point clouds. More correspondences adds more constraints which can reduce drift in feature-poor environments. Another goal is temporal redundancy, in the sense that sudden occlusions or spurious correspondences from moving objects impact the odometry estimate less when multiple key frames are used.

Some examples of registration-based radar odometry methods include continuoustime ICP [127], power-shifted NDT [615], and CFEAR [8] (not to be confused with the CFAR method for filtering which is discussed in Section 9.1.4) – all of which make use of submaps or multiple key frames. In CFEAR, point clouds are extracted from radargrams by selecting the k strongest returns along each azimuth. Each new cloud is registered jointly against the s most recent key frames using either a point-to-point, point-to-line, or point-to-distribution error metric —akin to NDT scan registration (see Part I). For each point, the normal vector is estimated from the covariance matrix of neighboring points within some radius. Point correspondences are weighted based on the agreement of their normal vectors, the planarity (condition number of the covariance matrix), and the number of points in the neighborhood. Kung et al. [615] use a fixed threshold to extract a point cloud from the radargram and aggregate multiple point clouds into a radar submap using an NDT representation where the contribution of each point is weighted by its returned signal strength. Burnett et al. [127] extract point clouds from radargrams using BFAR [25] and aggregate into a local submap, after which a continuous-time ICP formulation is used to optimize a trajectory estimate where points are associated with a Gaussian process motion prior.

The methods above are all based on 2D radar data. Recent pipelines for registrationbased 3+1D radar odometry tend to adopt similar strategies —although point cloud extraction is performed on the sensor so the design choices for selecting which signal peaks to consider as valid targets come down to sensor-specific thresholds rather than explicit feature extraction. Since per-point Doppler speed information is available in 3+1D point clouds, these methods typically consider a least-squares estimate of the ego-velocity from Doppler data (see Section 9.2.1) as an initial estimate and perform point cloud registration to refine it. The registration-based odometry component in 4DRadarSLAM [1271] uses a variant of GICP [985] that is adapted for radar point clouds, where points are weighted by a covariance matrix that assigns a higher uncertainty to points far from the sensor due to the limited azimuth and elevation angle accuracy. 4D iRIOM [1310] employs one-to-many distribution-todistribution matching in order to alleviate the noise and sparseness of radar point clouds. Instead of matching each point to its closest corresponding distribution in the local submap, each point is matched to a weighted set of closest distributions. The complete SLAM pipelines presented in [1310, 1271] are further described in Section 9.4.2. The EFEAR-4D method [1199] extends CFEAR 2D odometry [8] to 3+1D radar point clouds. After computing a Doppler-based ego-velocity estimate and removing outlier points that do not agree with this least-squares estimate and therefore can be assumed to come from moving obstacles, the remainder of the registration scheme is similar to CFEAR: registering scans to a sequence of preceding key frames and using agreement of normal vector and planarity for associating and weighting individual point matches.

## 9.2.5 Motion Compensation

As discussed in Section 9.1.3.3, it is important to compensate for motion distortion in odometry estimation. In the case of a low-speed spinning radar with a frequency of 4 Hz, egomotion compensation is reported to reduce ATE (Absolute Trajectory Error) by 29% by using a constant velocity model [8]. Given the time stamps of two subsequent radar scans and the relative pose computed using the methods above, a velocity can be computed and each radar point can be shifted accordingly, since the per-point timing is also available. Ofsetting the points of individual radar scans in such a way compensates for the substantial motion that can be encountered during the slow sweep of a scanning radar. The same model is also used to provide an initial estimate to rigid scan registration, and after registering each motion compensated scan to the previous, it is added as a node in the SLAM pose graph.

However, this approach still works in a discrete pose graph setting. Continuoustime trajectory representations can be beneficial for obtaining a smooth and accurate trajectory where the pose estimate used for undistortion can be queried at the time of each sensory reading. In Ng et al. [803] a spline representation is implemented in a pipeline that uses automotive SoC radars. Gaussian processes are used in Burnett et al. [129] to form a factor graph representing the trajectory by combining an IMU sensor with a spinning radar.

![](images/9b9170bb16405d02fa667a5307686fd19a220b65332e68c75a974685f15e8871.jpg)  
Figure 9.10 Two radar scans obtained from the same location. The noise pattern induce visual aliasing, complicating the process of place recognition.

## 9.3 Radar Place Recognition

As with other sensor modalities, place recognition (PR) is an essential module for radar SLAM. A good overview of the problem in general can be found in Chapter 8. As discussed in the chapters on visual and LiDAR PR, securing invariance to both translational and rotational change is crucial, such as when traversing a street in a diferent lane or arriving at a junction from another street.

## 9.3.1 Unique Challenges in Radar Place Recognition

While, in principle, several of the methods for PR discussed in the earlier chapters could be applied to radar data after proper data format conversion, some unique characteristics and challenges exist (see also Section 9.1.3).

Several factors contribute to the key challenges in radar place recognition . Firstly, the low resolution of radar data results in less details and thus fewer distinguishable features for recognition. The wide beam of radar data contributes to angular ambiguity, causing distant objects to appear as broad patches that difer significantly when viewed from closer ranges. Additionally, a relatively low signal-to-noise ratio poses challenges for place recognition if adequate filtering is not applied, as demonstrated in Figure 9.6. Receiver saturation can produce a strong radial feature that varies significantly with the observation angle of a particular target, as illustrated in the sample image in Figure 9.10. Consequently, the appearance of a place can change notably from nearby positions due to these challenges.

As there are diferent types of radars, place recognition needs to be handled diferently based on the type of sensor. How a place is perceived and described significantly difers between a long-range 360-degree spinning radar and a SoC radar with limited range and FOV. For example, a spinning radar produces a 2D radargram, which can be treated as an image. Naturally, approaches inspired by visual

PR have been applied for the spinning radar. Applying a target detection algorithm such as CFAR to the radargram results in a point cloud, after which methods from 2D LiDAR PR can be adapted. Spinning radars tend to have a very long range which can be greatly beneficial for place recognition [567] yet their slow scanning rate may lead to motion distortion even at low driving speeds. SoC radars, on the other hand, have a faster update rate since they need not mechanically spin the antenna in order to cover their field of view, and are therefore less susceptible to motion distortion. Most methods that use SoC radar data for place recognition work with a 2D/3D point cloud data format and not the full datacube. As a result, SoC radar PR often builds upon existing LiDAR PR methods; however, the measurements from SoC radars are typically restricted to a small FOV and tend to have higher sparsity and noise.

Currently there is more PR literature using spinning radars than SoC radars. This is in part because spinning radars capture richer information over a wider FOV, allowing them to better capture surrounding structures for robust PR, especially for outdoor applications, and in part due to the availability of large-scale datasets. However, this advantage does not prevent SoC radars from being used for PR. While spinning radars ofer a larger FOV, their projection model only provides 2D information, losing elevation details. For indoor PR, where a shorter range and smaller FOV are suficient, the 3+1D measurements from SoC radars can still be advantageous.

## 9.3.2 Learning-based Radar Place Recognition

For spinning radars, treating the 2D radargram as an image, 2D image retrieval has been leveraged for place recognition.<sup>3</sup>

Saftescu et al. [963] is an early approach to PR from 2D FMCW radar which uses a CNN to represent radargrams, specifically addressing their polar nature by using cylindrical convolutions in order to learn a representation that is rotation invariant. Then, a query radargram can be matched against a database of reference images to produce an appearance-only topological PR system. De Martini et al. [246] add pose refinement to produce a topometric mapping and localization system.

PR methods may be trained with augmented instances of the input data in order to be more robust to slight variations. Given that PR data typically is recorded sequentially, scan by scan while driving along a path,“augmented” instances can be retrieved by sampling frames that are sequentially nearby and artificially adding rotation by shifting the polar radargram. Contrasting to such augmented instances, the network is trained not only to recognize similar instances but also to distinguish instances (and their augmentations) that are sequentially far away. In this way, data for training a PR algorithm can be obtained in an unsupervised way, without knowing the true metric location of the radar data [352].

The methods mentioned above have been designed specifically for spinning radar providing 360-degree long-range coverage. SoC radars, which are more attractive for automotive applications given that they lack moving parts and have a smaller size, require slightly diferent treatment since they typically have lower range and smaller field of view.

Cai et al. [139] use a deep spatiotemporal encoder (after projecting the point cloud to a 2D image plane) to generate feature vectors as place descriptors for topological PR. These vectors are passed through a NetVLAD [35] layer after being re-ranked based on the RCS to filter out non-relevant stationary features. In this case, multiple radar sensors are mounted around the vehicle to overcome the limited field of view.

Herraez et al. [454] demonstrate single-scan radar place recognition with a pipeline that addresses data sparsity and noise by learning to focus on salient points that are important for place recognition. Similar to Cai et al. [139], Herraez et al. [454] also leverage NetVLAD to generate a feature encoding. However, they capture 3D contextual information by using rigid kernel point convolutions, as opposed to projecting the 3D point cloud to a 2D image. Furthermore, a ‘point importance estimator’ outputs the probability of a point being important for place recognition. This estimator is trained with sets of known corresponding point clouds, and query points that have a correspondence within a small radius in the other scan are labelled important. RCS information is incorporated through an separate network that encodes RCS data from the points into a more compact feature representation. Peng et al. [858] filters noise points in a similar way as the Doppler odometry methods in Section 9.2.1. Using RANSAC to estimate the ego-velocity produces a set of inlier points which are likely to be stationary. The remaining outlier points can then be filtered as noise. Rather than using NetVLAD which was designed for visual place recognition, Peng et al. [858] demonstrate a radar-specific feature extraction backbone named MinkLoc4D which takes inspiration from LiDAR place recognition architectures [1320].

## 9.3.3 Descriptor-based Radar Place Recognition

Using hand-crafted descriptors instead of learned embeddings is often efective as well. A common approach for radar PR is to directly adopt LiDAR descriptors, such as Scan Context [568], RING [1212], or M2DP [445]; though proper adjustments are necessary due to the difering sensor data formats. For example, the 3D structural information (e.g., height) is missing in spinning radar data, but can be replaced with RCS. Furthermore, additional care should be taken with radar sensor data to address its inherent challenges, often requiring careful noise filtering, sparsity handling and motion compensation.

Hong et al. [474] implement place recognition by adapting the M2DP descriptor [445] originally designed for 3D point clouds to 2D point clouds extracted from radargrams. Additionally, they investigate the distribution of points on the 2D plane to assess if a point cloud is likely to be distinctive or not. Performing PCA on the 2D points produces two eigenvalues that describe the spread of the points along each eigenvector. Point clouds where the eigenvalues are substantially diferent indicate cases where the scan lacks features in one direction (such as data from highway driving) and those scans are not considered for place recognition by Hong et al.

Jang et al. [510] modify the RING descriptor [1212] for radar. The descriptor generated by RING is in a sinogram form providing roto-translation invariance. The correlation between two sinograms should give the correct match for the PR problem; yet, the high level of noise in radar images may prohibit a naive comparison. Additional incorporation of auto-correlation has been shown to enhance the PR performance for radar images.

Adolfsson et al. [9] adapt Scan Context descriptors made from 2D radargrams in several ways. Firstly, they compute the sum of intensities for all points in a Scan Context bin as a way to encode both the intensity and the point density. Further, each descriptor is generated from an aggregated set of noise-filtered and motion-compensated polar images in order to mitigate some of the challenges listed in Section 9.3.1. Keeping only the k strongest returns along each azimuth direction provides a conservative filter that tends to remove a large part of the noise otherwise present in radar point clouds. Creating the Scan Context descriptor from multiple registered point clouds further addresses the sparse data remaining after this conservative filtering. De-skewing the point cloud via a constant acceleration model is important for generating comparable descriptors when using spinning radars while driving.

PR methods designed for 3+1D SoC radars also commonly implement a variant of the Scan Context descriptor [1271, 1310, 658]. However, as spurious radar points can easily distort the height measurements, using the maximum height per radial bin (as in the original Scan Context) is not always as efective for radar point clouds as for LiDAR. The Intensity Scan Context descriptor [1151] is an alternative that stores the maximum measured intensity value of the points in a Scan Context bin rather than the height. Alternatively, the sum of intensities within a bin can be used, so as to use both the intensity and the point density, thus being able to encode vertical structures in the descriptor without being as susceptible to noisy point positions. Another important factor when used with 3+1D radar is that the modified radar descriptor should cope with a much narrower FOV compared to the 360◦ LiDAR that Scan Context was designed for. Given that a place will appear quite diferently when observed from two diferent viewpoints, it is dificult to achieve rotation invariance when the sensor only covers a small FOV. One way to address this is to apply loop pre-filtering based on the current odometry estimate, only attempting to match the current descriptor to those of frames within a certain range of yaw angles (e.g., 20◦) [1271].

## 9.4 Radar SLAM

Radar SLAM systems generally implement the same overall structures as LiDARor camera-based SLAM solutions. In this section we briefly describe notable systems from the past two decades with a focus on aspects that are particular to tailoring SLAM to radar data. We also describe multi-modal systems that combine radar with other exteroceptive sensors in Section 9.4.3.

## 9.4.1 Map Representations

Many radar SLAM systems generate maps that rely on similar representations discussed in Chapter 5. However, the sparse and often spurious nature of radar measurements introduces unique challenges in the mapping process. Some earlier works in radar mapping establish a map representation directly from existing target detection models, often using landmark maps where individual detected targets constitute the map [224, 271, 140, 983]. This target detection can also be used for occupancy grid map as in the series of works by Mullane et al. [781], while exploiting the detection probability [782] in the mapping phase.

A detected target, represented by an individual peak in the signal, can also be treated the same way as a point from a LiDAR scan. For instance, these points can be used to create 2D occupancy grid maps [730, 762] or point cloud maps [474]. The extracted point cloud can also be augmented with additional information, as in [9] which also computes the distribution of surrounding points to estimate orientation and weights, similar to surfels or NDT cells. Direct point cloud representations are also popular in emerging high-resolution SoC radars, which feature a larger vertical field of view (3+1D) and a larger number of TX/RX antennas (e.g., 48 TX + 48 RX antennas for the Sensrad Hugin radar) [1271, 1310].

While less common, the full radar heatmap – comprising intensity samples for each direction and range bin prior to peak detection – can also be used to provide a dense grid map [950]. In this case, the map represents a global heatmap of the reflected power at each point in the environment, rather than evidence of occupancy. The 2D alignment between these heatmaps is then achieved through correlation.

Kramer and Heckman [604], in addition to generating odometry, presented a novel sensor model for voxel based mapping of radar data, capable of creating sparse maps even through visual occlusions. The sensor model leverages the log-odds based estimation of occupied vs free cells used in Octomap [478], but replaces their ray-cast model. The Octomap ray-cast model assumes that the first contact of a sensor is the only relevant point of occupation, but the generalized model accounts for radar’s ability to penetrate certain material types by updating voxel probabilities within the sensors field of view, increasing probabilities in cells with radar returns and decreasing probability with missed scans, without assuming information along a ray. A related grid-map representation specifically designed with radar in mind is due to Nuss et al. [815] who designed a state estimation filter to address dynamic obstacles in grid maps called a probability-hypothesis-density multi-instance Bernoulli filter. This filter casts grid cells as a finite stochastic set, and fuses radar and lidar data dynamically.

Lastly, the challenges of lower density and higher noise in radar data can be addressed using machine learning techniques [1210, 773], where LiDAR data serves as ground truth to achieve higher resolution and reduced noise in radar outputs. Mopidevi et al. [773] build a global radar map, where patches of the map are upscaled using a predictive network that filters noise from free-space regions and fills in sparse and empty regions, generating a map more similar to what can be obtained with LiDAR data.

Recently, neural fields, originally developed for RGB data [765] and later applied for LiDAR data [1293, 1056] have been applied to 2D radar data as well [98]. The key feature of neural representations is that they implicitly represent the environment such that the neural network can be queried with a point in space and return a quantity such as the distance to the closest surface, the colour and opacity, etc. In the radar fields of Borts et al. [98], a physics-informed radar sensor model as used to create an implicit neural geometry and reflectance model which can then be used to synthesize radar measurements from unseen view points. The received power at the radar sensor depends on the known transmit power and antenna gain but also the RCS which is composed from the size, radar reflectivity and directivity of the object. The neural representation learns to decompose the measured RCS into size (area) on the one hand and the product of reflectivity and directivity on the other hand.

## 9.4.2 Radar SLAM Pipelines

In the preceding sections we have covered the main components that make up graphbased radar SLAM frameworks: open-loop odometry estimation, place recognition for loop closure detection, and map representations —in addition to some of the physical and technical principles that are pertinent to radar SLAM.

This section reviews a number of complete SLAM pipelines that use radar as the only exteroceptive sensor and discusses how they implement the components. These pipelines generally follow the same architecture as shown in Figure 9.1, with a frontend that has a filtering process to the raw radar data into a point cloud and a place recognition module to identify loop closures and add the corresponding constraints to the underlying pose graph , and a back-end mapping module that performs frame-by-frame odometry and SLAM-proper module that globally optimizes the map when loops have been detected.

![](images/9a32bcfd51412b64c55ffd3217286ac6c3d0bd86c4a1ea92a83e0c1c0d314fa4.jpg)  
Figure 9.11 Example 3D map produced with radar SLAM using 3+1D radar (Sensrad Hugin) and IMU input. Color denotes height. Left: before loop closure. Note the accumulated horizontal and vertical drift that is evident in in the left part of 3D map. Right: after loop closure.

Working with 2D radargrams from a spinning radar, the TBV-SLAM (“trust but verify”) pipeline [9] builds upon the CFEAR 2D radar odometry method discussed in Section 9.2. The pose of the sensor is tracked using every radar scan in sequence but in the interest of eficiency, a sparser set of key frames is included in the SLAM pose graph . Once the estimated traveled distance exceeds a certain threshold (e.g., 1.5 m) a new key frame is added to the pose graph and an odometry constraint is created based on the alignment to the latest key frame. As usual, a constraint in the graph requires both the relative pose ofset between the two nodes and the associated uncertainty expressed as a covariance matrix. Interestingly, it has been shown [9] that using a predefined diagonal covariance matrix with small values performs better than estimating the covariance based on the Hessian of the registration cost function, which might otherwise be expected to better capture the uncertainty stemming from the shape of the input point clouds such that a pair of point clouds from a tunnel would give a larger uncertainty (along the tunnel’s direction), for example. However, using the Hessian tends to under- or overestimate the uncertainty which may cause the back-end optimization to slightly misalign the key frames. A central part of the TBV-SLAM pipeline is the place recognition module, where several candidate loop closures are retrieved (“trusted”) and later tested after which the verifiably best candidate is selected. The Scan Context descriptor [563] (see 9.3.3) is adapted to account for both point density and signal strength (in lieu of the height data that is unavailable in 2D radar). Additionally, several techniques are implemented for retrieving and verifying loop candidates. For each key frame, several augmented descriptors are created by shifting the point cloud by lateral translation ofsets. When searching for loop closures, the query descriptor is matched to all the augmented descriptors currently in the database, in order to account for loop closures where the vehicle is driving in a diferent lane. Loop candidates found by matching descriptors in this way are then filtered based on the odometry estimate (which hinges on having accurate enough odometry over large distances). After propagating the uncertainty from the odometry constraints between the query and candidate, candidate loop closures where the two scans are estimated to be far apart can be discarded. However, jointly considering the descriptor similarity and odometry uncertainty further improves the robustness of loop retrievals. That is, instead of finding the most similar candidate c to a query descriptor $q$ such that $c = \mathrm { a r g } \mathrm { m i n } _ { c } d _ { \mathrm { d e s c r i p t o r } } ( q , c )$ and then filtering based on odometry, the candidate is found from $c = \arg \operatorname* { m i n } _ { c } d _ { \mathrm { d e s c r i p t o r } } ( q , c ) + d _ { \mathrm { o d o m e t r y } } ( q , c )$ ， where $d _ { \mathrm { o d o m e t r y } }$ is computed as the likelihood of frame $q$ being at the same place as c taking the accumulated odometry uncertainty into account. Pairs of radar scans thus found are then aligned with the CFEAR registration module and finally, an alignment verification module which includes overlap measures as well as the CorAl [7] measure trained to detect slight misalignments. All in all, this pipeline demonstrates a number of techniques used to adapt descriptor-based place recognition to radar data (taking into account the multiple signal returns available in a 2D radargram and compensating for the comparatively sparse and slow scanning) and to sift through multiple loop candidates in order to verify the best ones based on geometric alignment as well as the front-end pose estimate.

The RadarSLAM pipeline of Hong et al. [474] is another prominent example of radar SLAM with 2D spinning radar data. In this pipeline, odometry (open loop pose tracking) is achieved by tracking key points detected directly in the radargram. A blob detector generates key points which are then tracked from frame to frame with a Lucas–Kanade tracker [707]. From the traveled distance computed by the tracker, a constant velocity model is used to compensate for the motion during one revolution of the scanner, and transformed key points are stored in the factor graph together with the poses of the key frames. Key frames are, as above, selected based on traveled distance. While pose tracking is done with a sparse set of key points, for loop closure detection denser point clouds are extracted from the radargrams. This point cloud extraction is done similarly as in TBV-SLAM [9]; however, instead of selecting the k strongest points per azimuth, RadarSLAM selects all points with an intensity higher than one standard deviation over the mean value. M2DP descriptors [445] are then created from the point clouds of the key frames, and loop closures are detected by matching frames with similar descriptors. As a safeguard against matching nondescriptive point clouds, RadarSLAM avoids selecting loop closures from key frames that are too elongated; i.e., where the two eigenvalues computed from PCA are markedly diferent, since such point clouds are expected to be from non-unique places like a highway section. No other loop verification is performed.

One example of a SLAM pipeline based on SoC 2D radar is due to Schuster et al. [983]. Diferently to the methods above, they maintain a graph that consists not only of the sensor poses but also individual radar feature nodes, whereas TBV-SLAM and RadarSLAM only optimize a graph of poses (although radar points are associated to the pose nodes in order to facilitate place recognition and rendering of the map). As 2D SoC radars generally provide far fewer detections than 360-degree spinning radars or 3+1D SoC radars, maintaining all detections in the optimizable graph is more feasible here. Edges are added in the graph to represent observations between all concurrently observed features. Their landmarks are extracted using a binary annular statistics descriptor (BASD [908]). As BASD is a compact binary descriptor, it is feasible to directly compare the descriptors of all feature points in a local region so as to associate recent features with those already in the map. Those features that pass a RANSAC outlier rejection stage are added as vertices to the graph, along with a pose node with odometry information from wheel encoders. Assuming moderate drift from open-loop tracking, no explicit place recognition step is included, but point features can be matched with the BASD descriptors after loop closure, after which the SLAM graph is optimized in the back-end.

Two recent approached to 6DOF radar SLAM with 3+1D SoC radar are 4DRadarSLAM [1271] and 4D iRIOM [1310], both using 3D radar point clouds as input, where the point detection (filtering of the datacube) is handled onboard the sensor itself, so CFAR or other peak detection is not explicitly included in the SLAM pipeline. Still, the input point cloud may contain a lot of noise points. The Doppler radial velocity information included in the 3+1D point clouds is exploited by both methods to filter points from moving objects. The vehicle’s ego-velocity can be estimated from linear least squares of the measured Doppler point velocities, and outlier points for which the velocity model does not agree are removed. iRIOM further denoises the point clouds by keeping as inlier points only those that have suficiently many neighbor points (within a fixed radius) and where those points are compactly distributed (considering the covariance matrix of their spatial distribution). The Doppler ego-velocity estimation is used as a prior to a scan-to-submap registration step. 4DRadarSLAM uses a variant of GICP [985] termed APDGICP where points are weighted by a covariance matrix that assigns a higher uncertainty to points far from the sensor due to the limited azimuth and elevation angle accuracy. The submaps (key frames) are inserted into a graph. Loop closures are detected by Scan Context matching, and as opposed to the 2D methods above, the original Scan Context descriptor that includes point elevation can be used. While 4D iRIOM uses the original Scan Context, 4DRadarSLAM uses Intensity Scan Context [1151] in order to avoid uninformative descriptors due to noisy elevation measurements. 4DRadarSLAM additionally includes a validation step to reject candidate matches returned by Scan Context if the accumulated odometry distance between the two frames is above a certain threshold.

## 9.4.3 Multi-modality in Radar SLAM

So far we have mostly been concerned with methods for radar SLAM that use only radar data and in some cases propriocetive sensing like IMU or wheel odometry.

In this section we discuss systems that combine mmWave radar with other exteroceptive sensors (e.g., LiDAR, camera) or external data (satellite imagery or prior maps).

Some works in radar mapping have employed radar–LiDAR fusion so as to generate the best possible set of points given the current visibility conditions (trusting LiDAR more in clear conditions and radar more in low visibility). Fritsche et al. [342, 343] fuse the sensors based on estimated ranges to determine which sensor to trust in a given case. Radar and LiDAR have also been combined to improve place recognition, overcoming diferent sensor modalities by registering radar to LiDAR maps [1247, 1248].

Doer and Trommer [273] extend ROVIO [90], which is a filter-based visual-inertial odometry approach, to integrate radar egovelociy estimates using the Doppler odometry approach described in Section 9.2.1 [272]. In a similar way, also thermal camera data can be fused with radar to achieve a multi-modal radar-thermal estimation pipeline [273]. Zhang et al. [1272] combine data from thermal camera and a 3+1D radar point cloud in order to get robust frame-to-frame odometry in low-visibility settings. A transformer-based feature matcher detects corresponding points in sequential thermal frames and the radar point cloud is used to improve the depth estimate.

In terms of using multi-modal data for radar-based navigation, it is also worth mentioning methods that make use of overhead images and road maps, although most works in the literature exploit this kind of data specifically for localization, rather than for full SLAM. Hong et al. [475] demonstrate how 2D scanning radar data can be used to localize in prior public maps such as OpenStreetMap. Their system runs odometry using RadarSLAM [474] and represents the estimated pose of the sensor as a Gaussian mixture model. Line segments corresponding to building walls are extracted from OpenStreetMap which is used as a prior. Oriented points that are extracted from the radargram are then matched to the features of the prior map. However, given that the prior map information is uncertain and may be incomplete or outdated, this point-to-feature data association is challenging. Poses are sampled from the Gaussian mixture model of the current pose estimate and for each pose the oriented points of the current radar scan are matched to the line features of the prior, which localizes the scan to the prior. Another method that uses prior map data, in this case satellite imagery, for localising 2D radargrams is RSL-Net [1069], which consist of a set of deep neural networks. The first generates a synthetic image from an input overhead photo, showing what a radargram from that place might look like. Another networks estimates the relative rotation between a real radargram and an overhead photo (via the synthetic radargram generated in the previous network). Finally, another network estimates the relative translation ofset between the radargram and the overhead image.

Some systems make use of so-called ultra-wideband (UWB) radio sensing. While UWB also uses electromagnetic waves within the radio spectrum and is sometimes referred to as UWB radar, the ranging capabilites of UWB difer dramatically from mmWave radar. When used in a radar SLAM framework, UWB radar is mostly used to detect similarities between sensor readings from diferent places. The frequency reponse after sending out a wide-band and wide-beam signal can provide a signature of the current location. The metric information is instead derived from wheel encoders or IMU data. Schouten and Steckel [979] and Takeuchi et al. [1065] use a database of UWB wave signatures to detect revisited places, For each place (node), a signature from the radar echo is stored along with the estimated pose. A graph is then created and optimized with odometry and loop constraints. Both approaches rely on odometry to obtain distances between nodes (i.e., edges in a graph) for metric SLAM and the signatures are created using a pulse-echo UWB sensor. Premachandra et al. [886] conversely use UWB radar to detect point features to be used in a landmark-based SLAM framework. They make use of multiple radar modules on each side of the robot and use trilateration of matched peaks in the signals from the sensors on either side to detect landmarks. In addition to the above, several UWB-based localization approaches use anchor-tag sensor configurations, where anchors are fixed to known locations and a battery-powered UWB tag is mounted on the robot or the asset to be localized. However, as this approach requires preinstalled infrastructure it is not directly related to SLAM.

## 9.5 Radar Datasets

In this section, we briefly summarize notable datasets from the radar SLAM literature. The datasets as also listed in Table 9.1.

Spinning Radar. The datasets with spinning radar in Table 9.1 all use Navtech 2D radar sensors. Two of the first large-scale radar datasets for odometry and SLAM are the Oxford Radar Robotcar dataset [54] and MulRan [567]. These datasets have both been quite well used in the literature. The Oxford dataset covers a set of traversals of an urban driving route, totaling 280 km in various weather conditions. The MulRan dataset covers a more diverse set of environments, both dense urban and more rural driving, and longer time spans between sessions, but less driving in total. MulRan focuses on facilitating PR research but has also been well used to benchmark odometry and SLAM methods. The Boreas dataset [128] includes data from driving a route repeatedly over the course of one year (385 km in total), notably including adverse weather conditions such as snow and rain. In addition to SLAM-related benchmarks, this dataset also includes benchmarks for object detection (cars, pedestrians, cyclists). The Oxford Ofroad Radar Dataset [353] is focusing on non-urban driving, in contrast to the other datasets in Table 9.1. This dataset covers about 154 km driving on unpaved roads and mountain trails in unpopulated areas.

SoC Radar. Several SoC radar datasets are also available, both with 2+1D and 3+1D data. Some datasets geared towards autonomous driving focus primar-

Radar SLAM

<table><tr><td colspan="2">Dataset</td><td>Lidar</td><td>Cameras</td><td>Ground truth</td><td>Environment</td><td>Inclement Weather</td></tr><tr><td rowspan="5">Spinning radar</td><td>Oxford Radar RobotCar [54]</td><td>Yes</td><td>Stereo/Mono</td><td>GPS/IMU + VO</td><td>Dense Urban</td><td>###, ###</td></tr><tr><td>Boreas [128]</td><td>Yes</td><td>Mono</td><td>GPS/IMU + RTK</td><td>Sparse Urban</td><td>###, *, ###</td></tr><tr><td>MulRan [567]</td><td>Yes</td><td>No</td><td>SLAM</td><td>Mixed Urban</td><td>-</td></tr><tr><td>RADIATE [1001]</td><td>Yes</td><td>Stereo</td><td>GPS/IMU</td><td>Mixed Urban</td><td>###, *</td></tr><tr><td>OORD [353]</td><td>Yes</td><td>Mono</td><td>GPS</td><td>Urban and Offroad</td><td>*, ●</td></tr><tr><td rowspan="8">SoC array radar</td><td>nuScenes [137]</td><td>Yes</td><td>Stereo</td><td>GPS/IMU</td><td>Mixed Urban and Natural</td><td>###</td></tr><tr><td>RadarScenes [982]</td><td>No</td><td>Mono</td><td>None</td><td>Mixed Urban Roadways</td><td>###, ###</td></tr><tr><td>ColoRadar [607]</td><td>Yes</td><td>No</td><td>SLAM</td><td>Varying</td><td>-</td></tr><tr><td>NTU4DRadLM [1273]</td><td>Yes</td><td>Mono</td><td>SLAM</td><td>Mixed Urban</td><td>-</td></tr><tr><td>MSC-RAD4R [212]</td><td>Yes</td><td>Stereo</td><td>GPS + RTK</td><td>Mixed Urban</td><td>#, *, ●</td></tr><tr><td>Snail [482]</td><td>Yes</td><td>Stereo</td><td>TLS</td><td>Roadways and Tunnels</td><td>###, ●</td></tr><tr><td>K-Radar [836]</td><td>Yes</td><td>Stereo</td><td>GPS/IMU + RTK</td><td>Roadways</td><td>###, *, ###, ●</td></tr><tr><td>TruckScene [324]</td><td>Yes</td><td>Stereo</td><td>GPS/IMU + RTK</td><td>Roadways</td><td>###, *, ###, ●</td></tr><tr><td>Both</td><td>HeRCULES [569]</td><td>Yes</td><td>Stereo</td><td>GPS/IMU + RTK</td><td>Mixed Urban and Natural</td><td>###, *, ●</td></tr></table>

<sub>(</sub>!<sub>: Rain</sub> <sub>: Snow</sub> <sub>: Night</sub> <sub>: Fog</sub> \<sub>: Smoke)</sub>

Table 9.1 Overview of public radar-related datasets. In the ‘ground truth’ column, VO denotes visual odometry, TLS denotes survey-grade terrestrial laser scans, RTK indicates GPS with real-time kinematic corrections.

ily on object detection but have also been used for developing and testing SLAM approaches. NuScenes [137] combines data from five Continental ARS408-21 radars mounted on the car used for data collection with one LiDAR and six cameras. The dataset focuses on urban driving, in four cities, and notably includes annotated labels for object detection of 23 object classes. RadarScenes [982] is a dataset with four 77 GHz automotive 2+1D radars (unnamed) and one camera. It focuses on semantic perception and contains labels for 11 object types, but lacks accurate ground truth as well as IMU and LiDAR data. ColoRadar [607] is a radar SLAM dataset with data from a 3+1D Texas Instruments MMWCAS-RF-EVM board as well as 2+1D Texas Instruments module, in addition to IMU and 3D LiDAR data. Notably, this dataset includes raw analog-to-digital converter (ADC) values from the radar sensors in addition to 3D ‘heat maps’ (data cubes) and individual point targets. It covers both indoor and outdoor data, as well as data from an underground mine, and includes 6-Degree of Freedom (DoF) ground-truth tracking for pose estimation. NTU4DRadLM [1273] and MSC-RAD4R [212] both include high-resolution 3+1D radar data from an Oculii Eagle sensor. NTU4DRadLM covers structured (university campus) and unstructured (park) environments and MSC-RAD4R covers urban and rural on-road driving. The Snail-Radar dataset [482] features two highresolution 3+1D radars: both Oculii Eagle and Continental ARS548, and includes data from handheld collection and on-road driving in urban environments.

## 9.6 Further Readings & Recent Trends

Radar SLAM pipelines that work in 3D are still rather few but as high-resolution SoC sensors develop we can expect to see more work on fully 3D odometry and place recognition with radar. This will be particularly important on drones, for example, where radar is less utilized today. Creative designs will be required to enable 3D wide field of view (FOV) radar units. In the meantime, we are likely to see significant advancements in handling constellations of several small FOV radars in SLAM. This naturally comes along with calibration concerns and other issues.

There will also likely be a shift towards making better use of the raw radargrams and datacubes, which contain spectral data. Traditional radar SLAM systems often resort to generating point clouds to interpret the environment. However, future systems are expected to take fuller advantage of the rich spectral information available in radar data, providing more detailed and nuanced maps and improving both object detection and classification. One challenge to this is convincing manufacturers to open up access to the raw output of their products for research.

Radar semantic segmentation may also play a more prominent role in place recognition. By leveraging segmentation techniques, SLAM systems can more efectively diferentiate between various types of objects in the environment, allowing for more intelligent navigation and decision-making. This will also help in reducing ambiguities in radar returns, leading to more reliable mapping.

Finally, there will be a greater focus on multi-modal data approaches, which integrate radar data with other sources of information including other sensors and also geographic priors (e.g., OpenStreetMap). For example, by combining radar observations with these priors, radar SLAM systems may be able to localize more accurately in large-scale outdoor environments, further enhancing the robustness and reliability of autonomous systems.

Event-based SLAM

Guillermo Gallego, Javier Hidalgo-Carri´o, and Davide Scaramuzza

An inquisitive reader would notice that SLAM is paramount in real-world applications that involve interpretation of spatial relationships and interaction with the environment. SLAM’s primary sensors are critical for the system’s success and adaptability. Visual SLAM is one of the most pervasive categories of SLAM methods because cameras are broadly available (afordable) and produce an intuitive and informative signal that allows the robot to sense the world in a wide range of scenarios $( e . g . ,$ , yielding lightweight systems that do not require additional infrastructure like GNSS). Despite the progress so far, state-of-the-art artificial vision systems are not as efective (robust and eficient) in real-world tasks as their biological counterparts. Standard cameras sense the world at a fixed frame rate that is independent of the scene dynamics. Thus, they become blind in the time between frames, introduce latency, potentially lose tracking, and produce large amounts of redundant data if nothing moves in the scene. This chapter pursues the visionary challenge of understanding and building visual SLAM systems that are fast (not limited by a frame rate), low-power, and robust to broad illumination conditions, by leveraging the bioinspired technology of silicon retinas or “event cameras”, which overcome several of the limitations of standard cameras (see Fig. 10.1).

We start by describing the working principles of event cameras (Section 10.1), as well as the corresponding challenges and applications (Section 10.2). Then we focus on methodologies to process event camera data (Section 10.3), and the corresponding front-end (Section 10.4) and back-end processing (Section 10.5). Finally, we discuss state-of-the-art systems (Section 10.6), datasets, simulators, and benchmarks (Section 10.7), and we conclude the chapter with a discussion about new trends and further readings (Section 10.8).

## 10.1 Sensor Description

## 10.1.1 Working principle

In contrast to traditional cameras, which acquire full images at a rate given by an external clock (e.g., 30 Hz), the pixels of event cameras like the Dynamic Vision

![](images/91588c3e5554144d7bbe083c816b89e4fde0c9d993f20d04a7c307245a6e036e.jpg)  
Figure 10.1 Drone with a downlooking DAVIS camera [111] (240×180 px) performing an autonomous flight using a visual-inertial odometry (VIO) algorithm [944] for state estimation. The high speed and high dynamic range characteristics of the event camera data are leveraged to operate in dificult illumination conditions. The insets show features (i.e., keypoints) detected and tracked in grayscale frames (left, motion-blurred) and in motion-compensated images of warped events (middle, sharp). The event data (in red/blue according to polarity) clearly respond to the scene contours. The same VIO algorithm [944] is also demonstrated on high-speed scenarios, such as an event camera spinning tied to a rope. Image from [361] (©2020 IEEE).

Sensor (DVS) [661, 361] operate independently from each other, responding to brightness changes in the scene asynchronously, as they occur (Figure 10.2b). These pixelwise changes are due to scene illumination (e.g., flickering lights) and/or to the relative motion of the camera and the scene (including moving objects). Hence, the output of an event camera is a sequence of digital “events” (or “spikes”), where each event represents a change of brightness (logarithmic intensity). This encoding is inspired by the spiking nature of biological visual pathways (Figure 10.2a).

Specifically, each pixel memorizes the logarithmic intensity L each time it sends an event, and continuously monitors for a change $\Delta L$ of suficient magnitude from this memorized value (Figure 10.2). When the change reaches a threshold $C .$

$$
\Delta L \doteq L (\boldsymbol {x} _ {k}, t _ {k}) - L (\boldsymbol {x} _ {k}, t _ {k} - \Delta t _ {k}) = p _ {k} C,\tag{10.1}
$$

the camera sends an event, $e _ { k } \doteq ( \boldsymbol { x } _ { k } , t _ { k } , p _ { k } )$ , which is transmitted from the chip with the $x , y$ pixel location $\scriptstyle { \mathbf { { \mathit { x } } } } _ { k }$ , the time $t _ { k }$ , and the 1-bit polarity $p _ { k } \in \{ + 1 , - 1 \}$ of the change $( i . e .$ , brightness increase or decrease). $\Delta t _ { k }$ is the time elapsed since the previous event at the same pixel.

Event cameras are data-driven sensors: their output depends on the amount of motion or illumination change in the scene. The faster the motion, the more events per second are produced because each pixel adapts its sampling rate to the rate of change of the intensity signal that it monitors.

![](images/942aafbf56ce3bb9e2cb412fca8648b0c57540084d1965a836e542ddcf415c74.jpg)  
Figure 10.2 Working principle of an event camera (e.g., DVS): (a) Three-layer model of a human retina and corresponding DVS pixel circuit; (b) Schematic of the operation of a DVS pixel, converting light into events (spikes), with the colors of the signals matching those of the layers in (a); (c) Comparison of the response of a standard camera and an event camera to a visual stimulus consisting of a black dot on a rotating disk. An event camera transmits the brightness changes continuously, forming a spiral of events in spacetime. Red color: positive events (ON spikes), blue color: negative events (OFF spikes). Images adapted from [884] (©2014 IEEE).

Bio-inspiration: The Transient Pathway. Event cameras are inspired by the operation of biological visual pathways, which are the information processing routes in animals and humans. Following the two-stream hypothesis, the dorsal stream (also called “transient” or “where” pathway) is dedicated to processing dynamic visual information (e.g., motion in the scene), whereas the ventral stream (called “sustained” or “what” pathway) is dedicated to object and visual identification and recognition. The DVS [661] corresponds to the part of the transient pathway from the photoreceptors up to the ganglion cells, adopting a simplified 3-layer pixel design that balances biological fidelity and circuitry stability (Figure 10.2). The three layers realize the functions of light conversion, delta-modulation, and comparison, respectively. Cameras like the Asynchronous time-based image sensor (ATIS) [883] or the Dynamic and Active-Pixel Vision Sensor (DAVIS) [111] model both visual pathways, and therefore output two types of signals: DVS events and grayscale information (e.g., images). More details of the main event camera types are provided in [884, 361].

## 10.1.2 Advantages of Event Cameras

The sensing principle of event cameras is radically diferent from that of standard (exposure-based) cameras that have dominated computer and robot vision for the last seven decades, and it ofers numerous advantages:

High Temporal Resolution: events are detected and timestamped with microsecond resolution, which enables capturing very fast motions without sufering from motion blur typical of frame-based cameras. Events are produced almost continuously in time, thus avoiding blind times that can cause large inter-image displacements and ruin data association in standard cameras.

Low Latency: each pixel works independently, without waiting for a global exposure time, thus events are transmitted as soon as a brightness change is detected, with submillisecond latency.

Low Power and Bandwith: events represent non-redundant temporal data, hence power is purposely spent. Bandwidth is also reduced (compared to a traditional camera operating at the same rate). At the die level, cameras consume less than 10 mW, allowing embedded systems to consume 100 mW or less [31].

High Dynamic Range (HDR): the range of light values that event cameras can sense is very high (typically >120 dB vs. 60 dB of standard cameras), enabling them to sense very dark (moonlight) and very bright (daylight) regions, simultaneously. Hence, they do not sufer from under/over-exposure typical of frame-based cameras. This property is due two facts: each pixel works independently and converts light to voltage in logarithmic scale.

## 10.1.3 Current Devices and Trends

Which event camera should I buy or use to solve my SLAM problem? We often get asked this question by people entering this emerging field. The characteristics of event cameras are often compared via tables [361, Table 1], [162, Tables 1–2]. Although multiple event camera designs exist, most of them are laboratory prototypes. Only a few make it into commercialized devices that enable the exploration of novel solutions to classical as well as new problems, such as event-based SLAM. Among the devices commercialized by the main manufacturers (SONY, Samsung, iniVation / SynSense, Prophesee, Omnivision), some trends are worth mentioning:

Pixel size: following the megapixel race of traditional cameras and pressure from industry requirements, the pixel pitch (i.e., size) has considerably decreased, from 40 µm (DVS128 [661]) to less than 5 µm [326]. DVS pixels carry out more operations (modulation, comparison, etc.) than their traditional counterparts; hence, they require more transistors, which are more dificult to pack in the same sensor area. To maximize the area of the pixels exposed to light (that is, the fill factor) and reduce the gap between the photoreceptive parts of the pixels, stacked technology and backside illumination have been adopted [326].

Grayscale output: early devices such as the DAVIS or ATIS concurrently output grayscale data (e.g., images [111]), which is especially useful in applications with stationary cameras (albeit this is not the usual scenario in SLAM). Newer models such as HD event cameras [326] discontinued the grayscale output in favor of more area for the event output, driven by the megapixel race.

Color is not essential in many motion-related tasks, and therefore only a couple of event camera models ofer color filters to detect changes in respective color channels (red, green and blue – RGB) [770].

Inertial data: some cameras also provide data from an inertial measurement unit (IMU) integrated in the same device. IMUs are valuable complementary proprioceptive sensors to cameras, enabling visual-inertial odometry (VIO) and SLAM, and yielding higher robustness and accuracy than single-sensor systems.

It is unrealistic to think that high-spatial–resolution event cameras are per se better than low-resolution ones. While capturing fine spatial details is important, noise and bandwidth also play an important role in the target application requirements. In SLAM and related tasks, where event cameras may move fast and/or over high-textured scenes, HD (1 Megapixel) event cameras can produce hundreds of millions of events per second. This poses problems, such as saturation of the output transmission bus of the camera, and high processing demands; currently there is no algorithm-and-hardware combination that can process such event rate in real time (without resorting to array-like conversion and/or sub/downsampling). New hybrid sensors, such as [1238], with lower spatial resolution for events than for intensity output, or foveated sensors [317], mimicking biological vision to decrease bandwidth), are being developed; they may provide alternative solutions to the above issue. In SLAM, a lower pixel resolution (e.g., QVGA) is preferred for algorithm prototyping and for real-time operation on computationally-constrained robots. Often the choice of field of view (optics) is as important as the pixel count.

## 10.2 Challenges and Applications

Event cameras represent a revolutionary technology in visual data acquisition. Hence, they pose the challenge of designing novel methods (algorithms and hardware) to process the acquired data and extract valuable information from it, unlocking the advantages of the sensor. In particular, the main challenges are:

Dealing with the space-time output: The output of event cameras is fundamentally diferent from that of standard cameras: events are asynchronous and spatially sparse, whereas images are synchronous and dense. Hence, visual SLAM algorithms designed for image sequences are not directly applicable to event data.

Dealing with motion-dependent data: Unlike images, each event contains binary (increase/decrease) brightness change information that depends not only on the scene texture, but also on the relative motion between the scene and the camera.

Dealing with noise and dynamic efects: Event cameras are noisy because of the inherent photon shot noise, transistor circuit noise, their dependency on the amount of incident light, non-idealities and low-power (sub-threshold) operation.

These challenges call for new approaches that rethink the space-time, photometric and stochastic nature of event data. In the context of SLAM, this poses questions such as: What is the best way to extract information from the events for pose or depth estimation? What map and camera trajectory representations shall be used that take into account the quasi-continuous temporal granularity and sparse nature of event data? How to establish correspondences (data association) under motiondependent data? How to model the problem (and its solution) without introducing the typical bottlenecks of frame-based technology?

The above questions have been driving the research on event-based SLAM (Fig. 10.3). This topic has evolved both on its own and in conjunction with other tasks, i.e., research on event-based SLAM has fostered research on other event-based tasks. For example, the synergy between SLAM and image reconstruction (the task of recovering absolute intensity from events) has been leveraged as early as the first works [229, 570] (rotational-motion SLAM) and [571] (6-DoF SLAM). Event-based SLAM and optical flow estimation have been treated together in [229, 1243, 1010, 585].

## 10.3 Overview and Taxonomy of Event-based SLAM Methods

Event-based SLAM methods can be broadly categorized in two classes, depending on how many events are processed simultaneously: (i) methods that operate on an event-by-event basis, where the state of the system (e.g., scene map and camera trajectory) can change upon the arrival of a single event, thus achieving minimum latency, and (ii) methods that operate on groups / batches / slices / packets of events, which introduce some latency. A key design choice in the latter category is how to select the size of the packet, for which many solutions have been proposed (e.g., fixed number of events, fixed temporal duration, and hybrid criteria).

Orthogonally, depending on how events are processed, model-based approaches and data-driven (i.e., machine learning) approaches can be distinguished. Mimicking the categorization in frame-based SLAM, event-based SLAM methods can be classified into indirect methods (feature-based, using event corners, lines, normal flow, etc.) and direct (using all events). This categorization is related to the type of objective or loss function used: geometric- vs. photometric-based (e.g., a function of the event polarity or the event rate/activity), and also to the overall philosophy: indirect methods typically have two steps (a feature extraction step, which “converts” events into geometric primitives, followed by a geometric SLAM pipeline), whereas direct methods typically comprise a single step that maps event data into motion and scene parameters. In the latter, the event generation model (10.1) (or its linearized version [361]) is a cornerstone for designing estimation methods. Handling data association between events is a central problem in event-based vision, and SLAM in particular. Due to the high temporal resolution of event cameras, data association is typically handled by temporal and spatial vicinity; both hardassociation and soft-association strategies have been explored.

![](images/b7983bc219141df3389377b2476099342af9addffe2ea6cfcbe86f439d06421d.jpg)  
EVO: A Geometric Approach to Event-Based 6-DOF Parallel Tracking and Mapping in Real-time

![](images/e3fa4b1556a576502a0314644f3c69b71f2c3f7189bb7be70dbf6ba272e15498.jpg)  
EDS: Event-aided Direct Sparse Odometry

![](images/477a06fdcd350a27d6c405e950e11b6d878d21971d30a8a34029e78bea0ae6fe.jpg)  
CMax-SLAM: Event-based Rotational-Motion Bundle Adjustment and SLAM using Contrast Maximization

![](images/3ad912df3bfa2c1bac48f0d4d2cff3aca5450cd138095caeb51c48dd085b30d7.jpg)  
Real-Time 3D Reconstruction and 6-DoF Tracking with an Event Camera (ETAM)

![](images/621be325dc394fd53b2995c35daa7fd6e13eecafd6801d3f2e95226f12b19e3b.jpg)

![](images/0301f74690d97f7919593d2469513012c6fd38f34e615c0dbf35488021e43c95.jpg)  
DEVO: Deep Event Visual Odometry

ESVO2: Direct Visual-Inertial Odometry with Stereo Event Cameras  
![](images/148811a6766bc0b1daedaa9842e64ac64d8fa32438351f6da9dad8f28cc22442.jpg)  
Event-based Stereo Visual Odometry via a Gaussian process continuous-time method (GPCT)  
Figure 10.3 Event-based SLAM is actively being investigated, with systems that explore a large variety of approaches, including classical methods and more recent deep learning solutions. Since events are triggered by moving edges on the image plane, it is natural to recover scene maps in the form of edges (e.g., sparse or semi-dense 3D maps). Images adapted from EVO [914] (©2017 IEEE), EDS [461] (©2022 IEEE), CMax-SLAM [416] (©2024 IEEE), Kim et al. [571] (©2016 Springer), ESVO2 [812] (©2025 IEEE), DEVO [585] (©2024 IEEE), and Wang et al. [1154] (©2023 IEEE).

Each of the above categories has advantages and disadvantages. The problem of solving SLAM with event cameras is challenging, and has been historically tackled by gradually increasing complexity along several axes: the number of unknowns (degrees of freedom – DoFs), the type of motion (from rotational or 2D scenarios to 6-DoF motion), the scene complexity (texture) and its motion (static vs. dynamic – independent moving objects – IMOs). Event-based SLAM is not an isolated problem: as mentioned in Sec. 10.2, it has connections with other problems (optical flow, tracking, segmentation, etc.), in stronger or weaker form depending on the assumptions or scenario considered. In addition to the above-identified trends, it is noticeable that early research has focuses on model-based methods, whereas more recent papers explore the possibilities that deep-learning–based approaches ofer.

## 10.4 Front-end of an Event-based SLAM System

Event-based SLAM systems often consist of several modules, which tackle smaller subproblems, such as feature extraction, data association, bootstrapping, pose estimation, depth estimation, etc. A primary division consists of the front-end and the back-end. From an input-output point of view, the front-end receives the raw sensor data (plus possibly auxiliary information, such as camera calibration) and outputs a set of event camera poses and scene map(s) (see Fig. 10.4). The backend refines these variables (i.e., the SLAM problem unknowns) to improve the fit between them and the sensor data. The back-end operates after the front-end, at a slower pace (depending on the number of variables involved) and can feed back its output to the front-end to help reduce drift and correct errors.

Therefore, the front-end converts the information from the sensor (e.g., photons) into geometric primitives (e.g., camera poses) and also photometric information (e.g., map appearance). This often comprises a step of “feature” or “information” extraction. Hence, the first challenge is to understand the information contained in the event stream and be able to extract it using methods that preserve the characteristics of the data (low latency, sparsity, HDR, etc.). Assuming constant illumination, events are caused by moving contours (edges). Therefore, we may consider a moving event camera as an asynchronous edge detector, which means that the SLAM problem is formulated in terms of scene contours (Fig. 10.3). This is a priori sensible because contours are the most informative regions of the image plane, allowing us to estimate retinal motion, from which 3D information is inferred. Each event consists only of a 4-tuple and is subject to noise, hence it carries little information; thus many events (e.g., thousands, millions) are needed to produce reliable estimates of quantities such as camera poses and scene maps. Extraction of information from the event stream depends on the task and on many design choices, such as the type of representation of the SLAM variables (scene map, camera trajectory), the hardware used to process the data, the output rate, etc.

Event-based SLAM  
![](images/929950bfdc51793cbf0de7c1a7d411a2ae132a68b1ef2d81b62147eed7a65542.jpg)  
Figure 10.4 Event-based SLAM pipeline with a front-end (that computes and a map and camera poses) and a back-end (that refines the map and poses). Since events respond to moving edges, the recovered map is often an edge (gradient) map. The example shows a direct, rotational SLAM pipeline (poses consists of rotations, and the map reduces to a panoramic map) [417]. An absolute intensity map may be recovered by Poisson integration.

## 10.4.1 Pre-processing and Event Representations

In the SLAM problem, the event camera continuously outputs data as it moves through the scene. Events are triggered “everywhere” on the image plane, as from the camera’s point of view it appears that all scene edges are moving. Since events are sparse and have microsecond resolution, each of them corresponds to a diferent camera pose. This is radically diferent from traditional (frame-based) cameras, where all pixel measurements of an image have the same timestamp and therefore share a common camera pose (this is the paradigm on which traditional multi-view geometry [437] has been built). Many SLAM methods convert event data into alterantive representations (event images, time maps or “time surfaces”, voxel grids, etc.) [361] for diferent reasons, such as compatibility with conventional computer vision methods, easier interpretation, etc. This conversion step often implies a quantization of the information (e.g., grouping events with similar timestamps) and/or a loss of the sparsity (e.g., zero-filling arrays at locations where no events happen).

Therefore, the study of event representations [361, 370] has gained attention. It is typically the first stage of the front-end and it highly influences later processing stages: events are converted into a more familiar representation (e.g., images) that are easier to work with (to feed to mature SLAM methods designed for traditional images, or to design learning-based methods based on images). This conversion is in part due to the fact that the research community is still exploring the best way to extract information from the event stream and tries to reutilize mature image-based methods. The front-end may use diferent event representations; for example, EVO [914] uses raw events for its mapping module (EMVS [916]) and event (edge-like) images for its camera tracking module. Ideally, one would design SLAM methods that use event representations that preserve the high speed and sparse properties of event cameras and do not sufer from the issues of traditional cameras (quantized time, latency, non-sparsity). In practice, this is an emerging research topic that requires rethinking visual processing asynchronously, and there is still ample room for improvement and investigation of fundamental results.

## 10.4.2 Indirect Methods

The design choices of the front-end largely influence the rest of the system. A major design choice is the type of processing method: indirect or direct. Indirect methods have broadly two steps; they first extract and track point-based, line-based or other type of feature from the events, and then leverage results from classical SLAM to estimate the camera motion and the structure of the 3D scene based on such geometric primitives. Features compress the event data into few informative primitives, which enables more efective and eficient use of the computational resources. A central problem consists in establishing and maintaining correspondences among the event features (and the map landmarks), which is known as data association. This is challenging, as each event carries little information and is motion-dependent to unambiguously determine association. Due to the high temporal resolution of event cameras, association can be established by spatio-temporal vicinity in pixel space. Hence, it is natural to track features rather than to match them.

Camera pose estimation or camera tracking is often formulated as the solution of a feature registration / alignment problem by minimization of a geometric objective (e.g., the reprojection error, measured using Euclidean distance in pixel space) given a map of the scene. The 3D structure of the scene is typically computed by means of triangulation (i.e., back-projection) of corresponding feature locations (e.g., to obtain 3D points and lines) using given camera poses. A large toolbox of mature geometric methods (multi-view geometry [437]) can be exploited.

Like conventional visual SLAM, indirect event-based methods rely heavily on robust feature extraction and tracking. However, these components are not yet as mature as their frame-based counterparts because they have to deal with unique challenges (large noise, sparsity, asynchrony, motion dependence, etc.). This limits the accuracy and therefore applicability of these systems. To address these issues, some systems resort to sensor fusion (with grayscale images and/or IMU data).

## 10.4.3 Direct Methods

Direct methods use all data available (not just the event data that conform to the definition of a feature) to estimate camera motion and 3D scene structure. They directly align event data with maps, images or other events without explicit feature extraction. If the event rate is high compared to the processing capacity of the system, data reduction mechanisms (e.g., denoising or subsampling) are adopted to reduce the number of events to process [416, 572].

As direct methods have only one step, the motion (camera tracking) or scene parameters (mapping) are obtained by optimization of some objective function (e.g., photometric error or spatial event rate error). The photometric-based objective induces a geometric registration objective. The problem unknowns are obtained by the alignment of edge-like brightness patterns conveyed by events and/or corresponding image or map pixels. Direct methods rely on the quasi-continuous nature of event data, for example to compute an incremental camera pose from the previously estimated one: the increment is small, as events are continuously triggered without gaps or blind times.

Among direct methods, a prominent subclass —due to their state-of-the-art accuracy performance— is that of methods that estimate motion or scene parameters by event alignment, which appears in the form of sharp images of warped events (IWEs). The idea is to estimate motion by “undoing it”, i.e., finding the parameters that motion-compensate the event data. Event alignment can be measured by means of diferent objectives: variance, gradient magnitude, disperson, etc. They are equivalently known as Focus or Contrast Maximization (CMax) [360, 1010]. In problems where events can be warped to a few pixels or a line, these methods can sufer from undesired global optima [1009]. Data association in direct methods is typically handled implicitly and in a soft manner, inherited by the distance in the pixel grid. Nevertheless, hard associations using nearest-neighbor values are also possible and efective in some cases.

## 10.4.4 Model-based and Learning-based Methods

So far, the majority of event-based SLAM approaches are hand-crafted, designed by human intuition based on the principles of operation of the event camera and the SLAM problem. Instead, deep-learning methods leverage artificial neural networks (ANNs) to model event data, either by converting events into image-like representations or by processing them directly with Spiking Neural Networks (SNNs). These methods are often categorized into supervised or self-supervised, depending on the type of supervisory signal. Self-supervised methods rely on events or other sensors (e.g., colocated grayscale images) to estimate depth and camera pose by leveraging some temporal dynamics or photometric consistency loss [1243]; whereas supervised methods require ground truth data for training [585], which is typically dificult to acquire in the real world. In recent years, many multi-modal datasets have been recorded onboard cars, drones, etc., which can provide the data needed for ANN training.

Learning-based solutions may substitute parts of the SLAM pipeline, such as feature extraction and tracking [760], or try to replace the entire system (endto-end). Learning-based approaches ofer the advantage of handling complex data representations and noise implicitly, but require large datasets for training and may sufer from generalization issues when applied to significantly diferent scenes (i.e., “domain shift”) from the ones in the training set.

## 10.5 Back-end of an Event-based SLAM System

The goal of a refinement module like the SLAM back-end [136] is to improve the consistency between the variables of the SLAM problem and the sensor data, thus improving accuracy and robustness of the fit, reducing the propagation of errors between tracking and mapping modules of the system. As we discussed in earlier chapters, bundle adjustment (BA) is commonly used in visual SLAM back-ends.

Event-based BA is still in its infancy, as most event-based SLAM systems lack a refinement step. Instead, they operate in a parallel tracking-and-mapping manner [571, 914, 1302], with each module relying on the output of the other concurrently running module as input to work properly. Current event-based SLAM systems have prioritized simplicity and taking advantage of the low-latency benefits of event cameras over accuracy and robustness. In addition to the challenges mentioned in Section 10.2 (noise, motion-dependent appearance, etc.), an event-based back-end poses the challenge of jointly estimating correlated variables, which implies a highdimensional search space, making optimization costly (in terms of complexity and latency) and prone to local minima.

Only recently BA has been used in systems that include event cameras. Since the back-end of a SLAM system is highly determined by the output of the front-end (as there needs to be a tight integration between both modules for best performance), we categorize event-based back-ends as indirect (feature-based) or direct (photometric-based).

Indirect back-ends are inherited from classical indirect frame-based methods [1107, 645, 787]. They operate on geometric primitives (corners, lines, etc.) that are detected in the event stream (possibly preceded by an events-to-image conversion [204, 944] to reutilize frame-based detectors). The objective typically consists in the minimization of the reprojection error, measured by the Euclidean distance in the image plane [437]. This approach has the advantage of reutilizing mature, robust techniques in classical SLAM. However, it discards the large amount of information contained in the events (as revealed by image reconstruction methods [918, 1280]) and often falls short of achieving the desired performance: due to noise and the dependency of events on motion, current event corners are not as accurate and stable as frame-based ones, hence their use in SLAM has been scarce [611]. Examples of indirect back-ends include [944, 204, 1154].

Direct back-ends work on sensor data (rather than geometric primitives) and the objective typically consists in the minimization of some form of photometric error. Hence they are more tailored than indirect ones. Approaches like [461], which leverage grayscale information from colocated frames, borrow the back-end from frame-based systems [308, 26]. However, grayscale frames can sufer from motion blur and low dynamic range. Event-only back-ends do not sufer from these limitations; they are recent and so far have been developed for constrained motions (planar or rotational). The objective may consist in the maximization of event alignment (also called motion compensation or CMax) [359, 416] or the minimization of the photometric error (i.e., temporal contrast) conveyed by each event [417, 418]. They are designed based on the event generation model (10.1). As each event carries little information and the number of problem unknows in SLAM is typically large, many events are needed for accurate BA, which poses demands on computational resources, power and latency. There is plenty of room for investigatation of eficient direct, event-only BA in natural scenes and 6-DoF motion scenarios.

## 10.6 State-of-the-Art Systems

Table 10.1 collects concrete systems in event-based VO/SLAM, describing some of their characteristics (direct, indirect, etc.) according to the categorization introduced in previous sections. While it is not possible to describe all of them in detail in this chapter (and neither is our intention), certain trends are worth mentioning.

The literature is dominated by model-based systems; data-driven approaches have not taken over yet (although that might happen in the near future, as it occurred with other computer vision tasks). Ever since the beginning, the problem of SLAM with event cameras has been tackled under diferent assumptions, increasing the complexity in terms of (i) camera motions, (ii) type of scenes, and (iii) additional sensors (or information, such as a map of the scene) to simplify the problem (e.g., a depth sensor attached to an event camera decreases the burden of depth estimation from events alone, and IMUs provide accurate angular velocity information, etc.).

Once an event-based method shows good performance, it is incrementally improved in an almost standard “exploitation” roadmap (similar to frame-based SLAM): for example, monocular methods [914] can be extended into stereo or multi-camera scenarios [381], event-only methods like [1302] (resp. [585]) can be robustified using inertial data fusion [685, 812] (resp. [408]), base system can be extended to handle omnidirectional lenses, etc. Despite this “exploitation” path, event-based SLAM is still an emerging field and, therefore, is in an exploration phase (of diferent techniques). This becomes evident when analyzing the methods in Table 10.1: diverse ideas and principles, leading to diferent map representations, event representations, loss functions, etc., are leveraged to design the estimation algorithms underpinning these systems. There is still plenty of room to investigate new state estimation ideas, especially those that take advantage of the genuine characteristics of the sensor.

## 10.7 Datasets, Simulators, and Benchmarks

Prototyping, training and benchmarking event-based vision systems places high demands for high-quality, diverse, and rich data (real and synthetic). The development of simulators, datasets, and leaderboards is essential to move the field forward and establish a common and solid ground in scientific and technical progress. Next we describe prominent SLAM datasets, benchmarks, and simulators for event cameras.

<table><tr><td>System</td><td>M/DL</td><td>I/D</td><td>Event represent.</td><td>BA</td><td>Motion</td><td>Scene</td><td>Input</td><td>Remarks</td></tr><tr><td>Cook [229]</td><td>M</td><td>D</td><td>Event Frame</td><td>✗</td><td>Rot</td><td>Natural</td><td>E</td><td>Interacting network using optical flow</td></tr><tr><td>Weikersdorfer [1175]</td><td>M</td><td>I</td><td>Individual Event</td><td>✗</td><td>Planar</td><td>2D B&amp;W</td><td>E</td><td>First filter-based Ev-SLAM.</td></tr><tr><td>PF-SMT [570]</td><td>M</td><td>D</td><td>Individual Event</td><td>✗</td><td>Rot</td><td>Natural</td><td>E</td><td>Two interleaved Bayesian filters</td></tr><tr><td>Censi [160]</td><td>M</td><td>D</td><td>Event Packet</td><td>✗</td><td>6DoF</td><td>B&amp;W</td><td>E+F+D</td><td>Filter-based VO based on image gradient</td></tr><tr><td>EB-SLAM-3D [1176]</td><td>M</td><td>D</td><td>Individual Event</td><td>✗</td><td>6DoF</td><td>Natural</td><td>E+D</td><td>Augment events with depth sensor</td></tr><tr><td>Yuan [1260]</td><td>M</td><td>I</td><td>Event Frame</td><td>✗</td><td>6DoF</td><td>B&amp;W</td><td>E+I+M</td><td>Vertical line-based camera tracking</td></tr><tr><td>Kueng [611]</td><td>M</td><td>I</td><td>Local Point Set</td><td>✗</td><td>6DoF</td><td>Natural</td><td>E+F</td><td>Event-based feature tracking VO</td></tr><tr><td>ETAM [571]</td><td>M</td><td>D</td><td>Individual Events</td><td>✗</td><td>6DoF</td><td>Natural</td><td>E</td><td>Three interleaved filters</td></tr><tr><td>CMax - $\omega$  [356]</td><td>M</td><td>D</td><td>Individual Events</td><td>✗</td><td>Rot</td><td>Natural</td><td>E</td><td>Contrast Maximization</td></tr><tr><td>EVO [914]</td><td>M</td><td>D</td><td>Edge Map</td><td>✗</td><td>6DoF</td><td>Natural</td><td>E</td><td>Event-event geometric alignment</td></tr><tr><td>EVIO [1303]</td><td>M</td><td>I</td><td>Point sets</td><td>✗</td><td>6DoF</td><td>Natural</td><td>E+I</td><td>Filter-based and MC features</td></tr><tr><td>Rebecq [915]</td><td>M</td><td>I</td><td>MC Event Images</td><td>√</td><td>6DoF</td><td>Natural</td><td>E+I</td><td>Feature-based, sliding-window back-end</td></tr><tr><td>RTPT [925]</td><td>M</td><td>D</td><td>Individual Events</td><td>✗</td><td>Rot</td><td>Natural</td><td>E</td><td>Panoramic tracker and mapper</td></tr><tr><td>Gallego [358]</td><td>M</td><td>D</td><td>Individual Events</td><td>✗</td><td>6DoF</td><td>Natural</td><td>E+M</td><td>Resilient sensor model</td></tr><tr><td>Mueggler [779]</td><td>M</td><td>D</td><td>Individual Events</td><td>√</td><td>6DoF</td><td>Natural</td><td>E+I+M</td><td>Continuous-time pose estimator</td></tr><tr><td>USLAM [944]</td><td>M</td><td>I</td><td>MC Event Images</td><td>√</td><td>6DoF</td><td>Natural</td><td>E+F+I</td><td>Sensor fusion &amp; sliding-window back-end</td></tr><tr><td>Chin [204]</td><td>M</td><td>I</td><td>Event Frames</td><td>√</td><td>Rot</td><td>Stars</td><td>E</td><td>Tailored to star tracking</td></tr><tr><td>ESVO [1302]</td><td>M</td><td>D</td><td>Time surfaces (TS)</td><td>✗</td><td>6DoF</td><td>Natural</td><td>2E</td><td>Stereo matching on TS patches</td></tr><tr><td>Hadviger [425]</td><td>M</td><td>I</td><td>Corners on TS</td><td>✗</td><td>6DoF</td><td>Natural</td><td>2E</td><td>Cross-corr. feature descriptors</td></tr><tr><td>CMax-GAE [572]</td><td>M</td><td>D</td><td>Individual Events</td><td>✗</td><td>Rot</td><td>Natural</td><td>E</td><td>Contrast maximization</td></tr><tr><td>EKLT-VIO [725]</td><td>M</td><td>I</td><td>Individual events</td><td>√</td><td>6DoF</td><td>Natural</td><td>E+F+I</td><td>EKLT tracker and VIO back-end</td></tr><tr><td>EDS [461]</td><td>M</td><td>D</td><td>Event images</td><td>√</td><td>6DoF</td><td>Natural</td><td>E+F</td><td>Frame-based back-end (DSO)</td></tr><tr><td>CB-VIO [684]</td><td>M</td><td>I</td><td>Individual events</td><td>√</td><td>6DoF</td><td>Natural</td><td>E+F+I</td><td>Feature tracker and VIO back-end</td></tr><tr><td>Wang [1154]</td><td>M</td><td>I</td><td>Binary images</td><td>√</td><td>6DoF</td><td>Natural</td><td>2E</td><td>Feature matching</td></tr><tr><td>El Moudni [303]</td><td>M</td><td>D</td><td>Time Surfaces TS</td><td>✗</td><td>6DoF</td><td>Natural</td><td>2E</td><td>Use ESVO tracker and EMVS mapper</td></tr><tr><td>ESVIO [183]</td><td>M</td><td>I</td><td>Time surfaces (TS)</td><td>√</td><td>6DoF</td><td>Natural</td><td>2E+2F+I</td><td>Feature tracking on from TS</td></tr><tr><td>ESVIO-direct [685]</td><td>M</td><td>D</td><td>Time surfaces (TS)</td><td>√</td><td>6DoF</td><td>Natural</td><td>2E+I</td><td>Extension of ESVO</td></tr><tr><td>PL-EVIO [410]</td><td>M</td><td>I</td><td>Time surfaces (TS)</td><td>√</td><td>6DoF</td><td>Natural</td><td>E+F+I</td><td>Point &amp; line features, sliding-window BA</td></tr><tr><td>CMax-SLAM [416]</td><td>M</td><td>D</td><td>Individual Events</td><td>√</td><td>Rot</td><td>Natural</td><td>E</td><td>Contrast Maximization refines motion</td></tr><tr><td>EVI-SAM [409]</td><td>M</td><td>D,I</td><td>Individual Events</td><td>√</td><td>6DoF</td><td>Natural</td><td>E+F+I</td><td>Dense mapping</td></tr><tr><td>Zuo [1317]</td><td>M</td><td>D</td><td>Individual Event</td><td>✗</td><td>6DoF</td><td>Natural</td><td>E+D</td><td>Augment events with depth sensor</td></tr><tr><td>DEVO [585]</td><td>DL</td><td>I</td><td>Event voxel grids</td><td>√</td><td>6DoF</td><td>Natural</td><td>E</td><td>Event-version of DPVO [1084]</td></tr><tr><td>EMBA [417]</td><td>M</td><td>D</td><td>Individual Events</td><td>√</td><td>Rot</td><td>Natural</td><td>E</td><td>Refines motion and gradient map</td></tr><tr><td>EPBA [418]</td><td>M</td><td>D</td><td>Individual Events</td><td>√</td><td>Rot</td><td>Natural</td><td>E</td><td>Refines motion and intensity map</td></tr><tr><td>ES-PTAM [381]</td><td>M</td><td>D</td><td>Events (also as frames)</td><td>✗</td><td>6DoF</td><td>Natural</td><td>2E</td><td>Use EVO tracker and EMVS mapper</td></tr><tr><td>ESVO2 [812]</td><td>M</td><td>D</td><td>Time surfaces (TS)</td><td>√</td><td>6DoF</td><td>Natural</td><td>2E+I</td><td>Extension of ESVO</td></tr><tr><td>DEIO [408]</td><td>DL</td><td>I</td><td>Event voxel grids</td><td>√</td><td>6DoF</td><td>Natural</td><td>E+I</td><td>Extension of DEVO and DPVO</td></tr></table>

Table 10.1 Summary of Event-based Visual SLAM methods, sorted chronologically. The columns indicate: the type of method (Model-based or Deep-Learning–based, Direct or Indirect), whether the method has a global refinement module (i.e., back-end / BA), the type of camera motions (Rotational, Planar, 6-DoF) and scenes (high-contrast black-and-white, etc.) it can handle, and the type of input data used (Event camera, Frame-based camera, Depth sensor, IMU and Map), where “2E” means stereo events (two sensors).

## 10.7.1 Simulators

There are a variety of publicly available tools for generating high-quality synthetic event camera data. ESIM [917] is an evolved version of [778], which was one of the first simulators to mimic the principle of operation of an event camera. Previous eforts, such as [535], just thresholded the diference between two successive frames to create edge-like images that resembled the output of an event camera. ESIM tightly couples the rendering engine and the event simulator, which allows the latter to adaptively render frames based on the dynamics of the visual signal.

The event camera simulator in CARLA [279] expands ESIM in more diverse, rich, and complex scenarios for autonomous driving. In the context of learning monocular depth from events [460], the event camera sensor developed in CARLA takes the rendered images from the simulator and computes per-pixel brightness changes to simulate an event camera in the same way as in ESIM. Figure 10.5 shows RGB images and events generated in the CARLA simulator.

![](images/4e7ca98446094e7873ed9e09fb222edf4f6e41a66b35fde68883d0b051c279cd.jpg)  
(a) CARLA simulator

![](images/53a5128d92d349e03120369137fef2a830d8b128d56580ea228bb9c929e80c95.jpg)  
(b) v2e simulator  
Figure 10.5 Event camera simulators: (a) RGB image and generated events using the ESIM simulator in CARLA [279]; (b) Detailed data processing steps of the V2E tool [481].

Motivated by learning-based approaches that require large amounts of event data for training and the fact that event data are hardly available due to the novelty of the sensor, a tool for converting any existing video recorded with a conventional camera into synthetic event data was developed: Video to Events (Vid2E) [371]. Hence, Vid2E aims at reducing the gap between publicly available datasets in traditional and event-based computer vision by enabling the use of a virtually unlimited number of existing video datasets for training networks designed for real event data. Vid2E solves the event data scarcity bottleneck by combining ESIM and adaptive video frame interpolation. ESIM can address this problem by adaptively rendering virtual scenes at arbitrary temporal resolution. However, video sequences typically only provide intensity measurements at fixed and low temporal resolution. Super SloMo [521] allows to reconstruct frames at arbitrary temporal resolution and then applies the event camera simulator ESIM. The number of intermediate frames is carefully chosen since, on the one hand, a low value leads to aliasing of the brightness signal, and on the other hand, high values impose a computational burden.

An important aspect of an event camera simulator is to accurately model noise, to reduce the simulation-to-real gap when transferring the algorithms. For example, ESIM, and therefore its derivative simulators, implement a simple noise model based on empirical observation [661]: the contrast threshold of an event camera (C in (10.1)) is not fixed but follows a normal distribution. To simulate this, at each step of the simulation, ESIM samples from a normal distribution $\mathcal { N } ( C , \sigma _ { C } ^ { 2 } )$ , where the noise level $\sigma _ { C }$ can be adjusted. Additionally, ESIM allows for separate positive and negative contrast thresholds $( C ^ { + }$ and C−) to more accurately simulate a real event camera. Other noise efects, such as spatial and temporal variations in contrast thresholds due to electronic noise or the limited bandwidth of event pixels, shall also be considered in event camera simulators.

Vid2E [371] models an ideal event camera lighting. V2E [481] proposes instead a more realistic noise simulator of an event camera based on the DVS circuitry. V2E is the first event camera simulator that includes temporal noise, leak events, and finite intensity-dependent bandwidth, including the same Gaussian threshold distribution as in Vid2E. V2E is a step forward towards a more realistic simulator enabling the generation of synthetic datasets covering a range of illumination conditions, which is an important use case for events. Similarly to Vid2E, V2E uses Super SloMo [521] to increase the temporal resolution of the input video. Figure 10.5 depicts the V2E architecture in detail.

Simulating event camera noises is a challenging topic for realistic synthetic event generation, EventGan [1306] proposes an end-to-end approach using deep learning to simulate event camera data. Their work proposes a method that leverages the existing labeled data for images by simulating events from a pair of temporal image frames, using a U-Net [934] encoder-decoder network. The methodology consists of training a neural network on pairs of images and events. Instead of applying a direct numerical error loss, they use an adversarial discriminator loss and a pair of cycle consistency losses. EventGAN generates a 3D spatio-temporal voxel grid for each polarity (instead of a set of individual events). This voxel grid representation is commonly used as input to ANNs.

VISTA 2.0 [30] is a simulator that integrates multiple sensor types, including RGB cameras, LiDAR, and event-based cameras, to facilitate policy learning for autonomous vehicles. It uses high-fidelity, real-world data to simulate diverse scenarios, such as varying weather, lighting, and road conditions. The event camera simulator works similarly to ESIM with adaptive sampling. The bidirectional optical flow between two consecutive frames is estimated using an ANN. VISTA 2.0 is designed for training perception-to-control policies, demonstrating enhanced robustness and sim-to-real transfer capabilities compared to real-world training data alone, thereby improving vehicle control in safety-critical situations.

Video to Continuous Events (V2CE) [1282] tackles the problem of producing events with more realistic timestamps than previous simulators. Vid2E and V2E generate events at discrete timestamps, instead of a continuous-time fashion like real events. This is not negligible in tasks that are sensitive to timestamp distribution, which prohibits the use of synthetic events since they can bring a significant domain shift with respect to real events. V2CE simulator works in two stages. The first stage consists of a supervised 3D U-Net encoder-decoder ANN that predicts two voxel grids (one per polarity, similar to EventGAN) from video. The second stage recovers precise event timestamps from the voxel grids. The method iteratively deduces the event count and their relative positions in each voxel. V2CE also shows that it can accurately generate events in saturated light areas and in edges where the event generation model for an ideal sensor does not hold.

## 10.7.2 Datasets and Benchmarks

The number of event-based datasets dedicated to Visual Odometry and SLAM has increased significantly since the publication of the ECDS [778] (see Table 10.2). ECDS was the first dataset with synchronized events, IMU, and ground-truth camera poses in 6-DoF. Previous datasets [955] included both synthetic and real events featuring pure rotational motion (3 DoF) in simple scenes with high visual contrast; ground-truth data was obtained using an IMU. Other work [55] enabled a 5 DoF comparison of event-based and frame-based camera movements; and ground truth was obtained from the pan-tilt unit encoders and the ground robot’s wheel odometry, making it prone to drift. ECDS contains hand-held, 6-DoF motion (slow- and high-speed) on a variety of scenes with precise ground-truth camera poses from a motion-capture system. The dataset consists of 11 scenes with real events and two additional scenes with synthetic events. The synthetic data was produced with the first version of what became the ESIM [917] simulator.

The RPG stereo dataset [1301] consists of eight hand-held sequences recorded with a stereo DAVIS [111] in an ofice environment and a synthetic sequence (featuring three fronto-parallel planes at various depths) produced by the simulator [778]. Although this dataset does not provide ground-truth depth, it has accurate ground-truth poses from a motion capture system and serves as a good starting point for prototyping and evaluating event-based stereo SLAM methods.

The Multi Vehicle Stereo Event Camera Dataset (MVSEC) [1304] is the first dataset to ofer ground-truth depth across a variety of platforms. It captures both indoor and outdoor scenes with varying levels of illumination and movement speeds. The platforms include a handheld rig, a hexacopter, a car, and a motorcycle, all equipped with calibrated sensors like 3D LiDAR, IMUs, and standard frame-based cameras. MVSEC has wide-ranging applications in pose estimation, mapping, obstacle avoidance, and 3D reconstruction, ofering accurate ground-truth depth and pose data through its integrated LiDAR system. The datasets contain long sequences that enable a comprehensive evaluation of event-based algorithms.

The UZH-FPV (First Person View) dataset [257] is specifically designed to advance research in autonomous drone racing. It features a custom-built quadrotor with a Qualcomm Flight Board and an mDAVIS346 [1074] event camera mounted on a Lumenier QAV-R carbon fiber frame. The recordings capture indoor and outdoor scenes at varying speeds and trajectories, which present challenges for navigation and state estimation. This dataset supports research in VIO, event data processing, and real-time drone applications in fast scenarios. It has become a key resource for developing high-speed camera motion algorithms, particularly in autonomous drone racing, and has also been used for competitions at conferences and workshops.

The Event Camera Motion Segmentation Dataset (EV-IMO) [769] is the first event-based dataset created specifically for segmentation of independently moving objects (IMO) in indoor environments. It contains 32 minutes of recordings, tracking up to three fast-moving IMOs using a motion capture system. The dataset provides pixel-wise motion masks, and ground-truth egomotion and depth. It is useful in robotics, especially in scene-constrained environments where accurate motion detection is crucial for tasks like object tracking and autonomous navigation. EV-IMO2 [126] builds on its predecessor by ofering more sequences, three higher quality event cameras, and more complex scenarios. This version serves as both a challenging benchmark for current algorithms and a rich training set for developing new methods, including event-based SLAM in monocular and stereo setups.

10.7 Datasets, Simulators, and Benchmarks

<table><tr><td>Dataset</td><td>Platforms</td><td>Pixel Resolution</td><td>Sensors</td></tr><tr><td>ECDS [778]</td><td>Hand-held</td><td> $240 \times 180$ </td><td>E, F, I</td></tr><tr><td>RPG-stereo [1301]</td><td>Hand-held</td><td> $240 \times 180$ </td><td>2E</td></tr><tr><td>MVSEC [1304]</td><td>Hand-held, Drone, Car, Byte</td><td> $346 \times 240$ </td><td>2E, 2F, I, Lidar, GPS</td></tr><tr><td>UZH-FPV [257]</td><td>Drone</td><td> $346 \times 260$ </td><td>E, F, I</td></tr><tr><td>EV-IMO [769]</td><td>Hand-held</td><td> $346 \times 260$ </td><td>E, F, I, Depth</td></tr><tr><td>EV-IMO2 [126]</td><td>Hand-held</td><td> $640 \times 480$ </td><td>3E, F, I, Depth</td></tr><tr><td>DSEC [372]</td><td>Car</td><td> $640 \times 480$ </td><td>2E, 2F, Lidar, GPS</td></tr><tr><td>TUM-VIE [584]</td><td>Hand-held</td><td> $1280 \times 720$ </td><td>2E, 2F, I</td></tr><tr><td>EDS [461]</td><td>Hand-held</td><td> $640 \times 480$ </td><td>E, F(RGB), I</td></tr><tr><td>VECtor [363]</td><td>Hand-held</td><td> $640 \times 480$ </td><td>2E, 2F, RGB-D, I, Lidar</td></tr><tr><td>M2DGR [1250]</td><td>Ground Robot</td><td> $640 \times 480$ </td><td>E, F, I, Lidar, GPS, Thermal</td></tr><tr><td>ViViD++ [635]</td><td>Hand-held, Car</td><td> $240 \times 180, 640 \times 480$ </td><td>E, F, RGB-D, Thermal, Lidar, GPS</td></tr><tr><td>FusionPortable [523]</td><td>Hand-held, Quadruped Robot</td><td> $346 \times 240$ </td><td>2E, 2F, I, Lidar, GPS</td></tr><tr><td>Stereo HKU-VIO [183]</td><td>Hand-held</td><td> $346 \times 260$ </td><td>2E, 2F, I</td></tr><tr><td>M3DE [163]</td><td>Drone, Car, Quadruped Robot</td><td> $1280 \times 720$ </td><td>2E, 2F, I, Lidar, GPS</td></tr><tr><td>CoSEC [861]</td><td>Car</td><td> $1280 \times 720$ </td><td>2E, 2F, I, Lidar, GPS</td></tr></table>

Table 10.2 Summary of event-based SLAM datasets, sorted chronologically. Same notation for sensor data as in Tab. 10.1. Stereo and multi-sensor datasets are further described in the survey [380].

The Stereo Event Camera Dataset for Driving Scenarios (DSEC) [372] is largescale, intended to support research in autonomous driving, especially in developing robust perception systems capable of handling adverse lighting conditions through sensor fusion of events and frames. The dataset features a platform with a multicamera setup, including two VGA-resolution event cameras (Prophesee Gen 3.1) with a 60 cm baseline and two RGB cameras (FLIR Blackfly S) with a 51 cm baseline (see Fig. 10.6). The setup includes a Velodyne VLP-16 LiDAR and an RTK GPS for precise localization. Data were collected in various urban and rural settings in Switzerland under diverse illumination conditions, such as day, night, and direct sunlight, providing ground-truth depth maps for stereo matching. DSEC also provides Optical Flow and Disparity to benchmark algorithms in challenging driving conditions. These benchmarks use metrics like N-pixel disparity error, Mean Absolute Error (MAE), and Root Mean Square Error (RMSE) to assess the performance of algorithms combining high-resolution event camera data with RGB frames.

The TUM Stereo Visual-Inertial Event Dataset (TUM-VIE) [584] employs stereo Prophesee Gen4 event cameras (1 Megapixel resolution) along with synchronized

![](images/1f06c772e4c615375e0cfdc49d460583ea8c2cd026670d5fb0ce699dc59a408a.jpg)  
Figure 10.6 Details of some Event-SLAM datasets: (a) the sensor suite mounted on top of a car, in the DSEC [372] dataset (©2021 IEEE). (b) Details of the beamsplitter that allows the sensors to share a spatially aligned field of view in the EDS [461] dataset (©2022 IEEE).

IMU data at 200Hz and stereo grayscale frames at 20Hz. It includes sequences from handheld and head-mounted setups in diverse indoor and outdoor environments, covering various scenes such as sports activities, HDR scenarios, and lowlight conditions. TUM-VIE is intended to facilitate research on VIO, SLAM, 3D reconstruction, and sensor fusion, especially in challenging conditions where traditional methods may fail, pushing the boundaries of high-resolution event-based perception algorithms.

The Event-Aided Direct Sparse Odometry (EDS) dataset [461] includes highquality events, color frames, and IMU data to support research in monocular VIO. Data were acquired using a custom-made beamsplitter device (see Fig. 10.6), allowing for precise alignment of RGB frames and events on the same optical axis, which is not commonly found in previous datasets. The scenes recorded include natural indoor environments, providing high-resolution, well-calibrated data for applications like optical flow estimation, depth estimation, and robust visual odometry under various motion and lighting conditions.

The Versatile Event-Centric (VECtor) Benchmark Dataset [363] is also designed to evaluate event-based SLAM algorithms. The recording platform holds a diverse sensor suite, including stereo cameras (event- and frame-based), an RGB-D sensor, a 128-channel LiDAR, and a nine-axis IMU, all mounted on a versatile 3D-printed holder. The dataset features both small-scale indoor environments, like a motion capture arena, and large-scale indoor environments with various complexities and illumination conditions. It claims to ofer high-resolution (VGA), synchronized data across diverse environments, ensuring reliable evaluation of SLAM algorithms in both static and dynamic, low-light, and HDR scenarios. This makes it a comprehensive resource for advancing research in multi-sensor SLAM applications.

The Vision for Visibility Dataset (ViViD++) [635] was recorded with a multisensor platform, including thermal cameras, to support research on SLAM algorithms that can handle poor visibility, motion disturbances, and appearance changes, leveraging the complementary strengths of diferent sensors. The Fusion-

Portable dataset [523] includes a Quadruped robot that moves in various scenes, such as corridors, canteens, roads, and gardens under diferent lighting conditions.

Finally, the Multi-robot, Multi-Sensor, Multi-Environment Event Dataset (M3ED) [163] (informally known as MVSEC 2.0) is focused on high-speed dynamic motions in robotics applications. It combines 1 Megapixel stereo event cameras, grayscale and RGB cameras, a 64-beam LiDAR, and high-quality IMU, all synchronized with RTK localization. Unlike previous datasets, M3ED ofers heterogeneous data from multiple platforms in both structured and unstructured environments, with ground truth pose, depth, and semantic labels, making it a valuable resource for developing robust event-based perception algorithms for dynamic environments beyond traditional driving or indoor applications.

## 10.7.3 Metrics

Ideally, SLAM systems should characterize the quality of their localization and mapping modules individually. However, because (i) both modules operate in an intertwined way (depth errors afect camera pose errors, and vice versa), and (ii) ground-truth localization information is considerably more compact (6-DoF) and easier to acquire than accurate ground-truth depth, the result is that depth estimation errors are often subsumed in the evaluation of camera trajectory errors.

Conceptually, since both classical SLAM and event-based SLAM output camera trajectories, event-based SLAM inherits the performance evaluation protocol from classical SLAM. Two commonly used metrics are the Absolute Trajectory Error (ATE) and the Relative Pose Error (RPE) [1045]. The ATE assesses the accuracy of the camera’s pose relative to a fixed world coordinate system; hence, it provides a broad assessment of the VO system’s long-term performance. The RPE evaluates the relative poses between consecutive (i.e., nearby) timesteps; hence, it focuses on the local consistency of VO system. The translational error in ATE, also known as positional error, is calculated as the Euclidean distance between the estimated and ground-truth camera positions. The rotational error, or orientation error, is determined by the geodesic distance in SO(3). Similarly, the translational and rotational parts of RPE are calculated between pairs of camera poses over a time interval. Some studies also compute the positional error relative to the mean scene depth or total distance traveled, ensuring that the error remains invariant to the scale of the scene or trajectory.

Additional error metrics —Average RPE (ARPE), Average Relative Rotation Error (ARRE) and Average Endpoint Error (AEE)— may be used to assess the estimated translation vectors and rotation matrices [1305]. Specifically, ARPE and AEE measure diferences in position and orientation between two translation vectors, while ARRE calculates the geodesic distance between two rotation matrices.

Beyond these metrics, average linear and angular velocity errors can also be useful for evaluating camera pose estimation, especially when working with event cameras, where abrupt and fast camera motions are estimated thanks to the events. Camera poses are functions of both velocities over time. Several toolboxes [405, 1283] are publicly available to facilitate the reproducibility of research and reduce the complication in SLAM trajectory evaluation.

In case depth estimation is evaluated separately, the average depth error at various cutofs up to fixed depth values is often used, allowing for comparison across methods on diferent scales of 3D maps. However, there are not many datasets that contain ground-truth depth information (see Sec. 10.7). The Root Mean Square Error (RMSE) of the Euclidean distance between the estimated 3D point with respect to the closest surface on the ground truth map is the preferred metric. Additional metrics, such as the Relative Error (REL) and completion (number of points recovered), are also used in the literature [302, 460, 381].

## 10.8 Further Readings & Recent Trends

Although research on event-based SLAM has made considerable progress, many open questions and problems remain given the novelty of the technology. These questions pertain to what are the best ways (hardware and software) to acquire and process visual information for a given task (e.g., SLAM) in order to rival or surpass (in terms of robustness, latency, eficiency, accuracy, etc.) the performance observed in biological species.

The sensor is asynchronous , but most of the systems in the literature are designed on serial (i.e., von Neumann) processors (due to the entry barrier to neuromorphic computing). This is suboptimal in terms of eficiency (power consumption), latency, etc. compared to the expected performance of fully neuromorphic systems [845], where event cameras are paired with asynchronous (brain-inspired, spike-based) processors, controllers, actuators, etc. It is a long-standing dream of the research community: to build robots that mimic the eficient processing of animals and their ability to map and localize themselves in the environment (with potential applications in “always on” inside-out tracking for AR/VR, etc.). This dream requires rethinking and co-designing sensors, processors, and algorithms [245] in a neuromorphic engineering way, which is very challenging, as it takes great breadth and depth of expertise, and coordination of multiple disciplines.

In the near future, novel hybrid sensors are being developed that provide data inspired by the two visual streams [1238], spatially and temporally aligned, with low latency, HDR, and fine details (pixel count). Alternatively, foveated sensors [317], mimicking biology, are also investigated to reduce bandwidth requirements. There is still a big field to explore in terms of event cameras, their evolution (e.g., nearsensor processors like pixel processor arrays, Aeveon sensors), and their combination with other sensors (frame-based cameras, structured light, LiDAR, radar, etc.) for data fusion and improved SLAM performance.

## Acknowledgments

The authors thank Giovanni Ciofi for his support in preparing this chapter.

Inertial Odometry for SLAM

Guoquan (Paul) Huang, C´edric Le Gentil, Teresa Vidal-Calleja, Davide Scaramuzza, Frank Dellaert, and Luca Carlone

Inertial Measurement Units (IMUs) have become one of the most pervasive sources of odometry for robot simultaneous localization and mapping. An IMU measures the linear acceleration and the rotation rate of the body the sensor is attached to. IMUs are available in a broad range of form factors, costs, and performance levels, from large and accurate optical sensors used on airplanes to small but more noisy micro-electromechanical systems (MEMS) used in smart phones and other consumer devices. The low-SWAP and inexpensive nature of MEMS IMU sensors renders them great candidates as sensors for robotics, where these sensors have been extensively studied with application to SLAM for more than two decades.

In this chapter, we first introduce basic facts about IMUs and describe their measurement model (Section 11.1). Then, we introduce the concept of IMU preintegration (Section 11.2), which allows adding high-rate IMU data into a factor graph optimization framework. Next, we observe that using IMU data introduces extra variables in the optimization (e.g., sensor biases) and discuss observability<sup>1</sup> properties of systems that combine IMUs with exteroceptive sensors, e.g., camera or LiDAR (Section 11.3). Finally, we showcase examples of what’s achievable with modern IMU-centric SLAM systems (Section 11.4) and review recent trends (Section 11.5).

## 11.1 Basics of Inertial Sensing and Navigation

A 6-axis Inertial Measurement Unit (IMU) comprises an accelerometer, which measures the linear acceleration of the sensor with respect to an inertial frame, and a gyroscope, which measures the angular velocity (or rotation rate) of the sensor.<sup>2</sup> Traditionally studied in aerospace engineering, inertial navigation systems (INS) aim at estimating the current state (e.g., pose, velocity) of the platform the IMU is mounted on, from the initial state and the history of the IMU measurements [171, 1099]. Diferent INS can be categorized into strapdown systems, where the IMU is mounted to the frame of the platform, and stabilized systems, where the IMU is mounted on an inner gimbal, multi-gimbal structure, or floating ball, which is designed to maintain its orientation constant with respect to an inertial frame. Most INS in robotics fall into the former category, i.e., they rely on an IMU that measures the local linear acceleration and angular velocity of the sensing platform it is rigidly connected to. In robotics, the term inertial odometry is commonly used as a synonym of inertial navigation, to emphasize the odometric nature of the estimate.

Clearly, the odometric estimate produced by an INS drifts over time, so in most applications the estimation also relies on other sensors (e.g., GPS, cameras, LiDARs), in which case one talks about aided inertial navigation systems (AINS). In robotics, it is common to directly specify the combination of sensors used with the IMU. For instance, a system that combines cameras and IMUs to provide 3D motion tracking is called a visual-inertial odometry, while a system that also includes loop closures is called a visual-inertial SLAM system.

## 11.1.1 Sensing Principles and Measurement Models

An IMU commonly includes a 3-axis accelerometer and a 3-axis gyroscope, measuring the angular rate and the linear acceleration of the sensor platform. The basic principle underlying gyroscope design is the conservation of angular momentum. On the other hand, an accelerometer uses the inertia of a mass to measure the diference between the kinematic acceleration with respect to the inertial frame and the gravitational acceleration. Diferent principles can be used for the design of accelerometers, for example, by using a rate gyroscope mounted as a pendulum mass, based on the inertia of a proof mass inside a low-friction case, or based on the diference in vibration of two thin metal tapes suspended inside a case with a proof mass suspended between them.

Measurement Model. We now describe the IMU measurement model, which relates the IMU measurements to the state of the robot and other quantities $( e . g .$ , biases) we need to estimate. For simplicity, we assume the sensor frame coincides with the body frame $\mathcal { F } ^ { b }$ of the robot, and the world frame ${ \mathcal { F } } ^ { w }$ is an inertial frame.<sup>3</sup> The IMU measurements collected at time t, namely ${ \bf a } ( t )$ and $\omega ( t )$ , are typically assumed to be corrupted by additive white Gaussian noise η and slowly varying

sensor biases b:

$$
\mathbf {a} (t) = \boldsymbol {R} _ {w} ^ {b} (t) \left(\mathbf {a} ^ {w} (t) - \mathbf {g} ^ {w}\right) + \mathbf {b} ^ {a} (t) + \boldsymbol {\eta} ^ {a} (t),\tag{11.1}
$$

$$
\boldsymbol {\omega} (t) = \boldsymbol {\omega} _ {b} ^ {b} (t) + \mathbf {b} ^ {g} (t) + \boldsymbol {\eta} ^ {g} (t).\tag{11.2}
$$

As usual, the superscript $\mathcal { F } ^ { b }$ denotes that the corresponding quantity is expressed in the body (IMU) frame $\mathcal { F } ^ { b }$ . The pose of the IMU at time t is described by the transformation $\{ R _ { b } ^ { w } ( t ) , p ^ { w } ( t ) \}$ , which maps a point from sensor frame $\mathcal { F } ^ { b }$ to ${ \mathcal { F } } ^ { w }$ (note that $R _ { w } ^ { b } ( t ) \ = \ ( R _ { b } ^ { w } ( t ) ) ^ { \top } ) ; \ \mathbf { a } ^ { w } ( t ) \in \mathbb { R } ^ { 3 }$ is the acceleration of the sensor in the world frame; $\mathbf { g } ^ { w }$ is the gravity vector in the world frame. Therefore, the term $R _ { w } ^ { b } ( t ) \left( { \bf a } ^ { w } ( t ) - { \bf g } ^ { w } \right)$ is the acceleration experienced by the IMU in the Body/IMU frame. The vector $\boldsymbol { \omega } _ { b } ^ { b } ( t ) \in \mathbb { R } ^ { 3 }$ is the instantaneous angular velocity of $\mathcal { F } ^ { b }$ relative to ${ \mathcal { F } } ^ { w }$ expressed in coordinate frame $\mathcal { F } ^ { b }$ . The noise terms $\eta ^ { g } ( t )$ and $\eta ^ { a } ( t )$ are assumed to be zero-mean Gaussian random variables, and the to-be-estimated biases $\mathbf { b } ^ { a } ( t )$ and ${ \bf b } ^ { g } ( t )$ are assumed to follow random walks. Note that here the superscripts for noise and bias vectors do not refer to the frames but the sensors (accelerometer and gyroscope); $e . g . , \mathbf { b } ^ { a } ( t )$ is the accelerometer bias.

Extended Models. While the IMU measurement model (11.1)-(11.2) often suffices in robotics, more sophisticated models that more accurately capture the sensing process may be needed, for example, when (re-)calibrating the sensor platform. Due to the imperfection in manufacturing, accelerometers may sufer from misalignment and scale errors, and the model (11.2) can be extended to:

$$
\mathbf {a} (t) = \boldsymbol {T} _ {a} \boldsymbol {R} _ {w} ^ {b} (t) (\mathbf {a} ^ {w} (t) - \mathbf {g} ^ {w}) + \mathbf {b} ^ {a} (t) + \boldsymbol {\eta} ^ {a} (t),\tag{11.3}
$$

where $\mathbf { \delta } _ { \mathbf { Z } _ { a } }$ is the shape matrix that models both misalignment and scale errors in the accelerometer measurements. Scale errors can be made of static or temperaturerelated components and can be determined during sensor (intrinsic) calibration. Similarly, the gyroscope measurement model can be extended to capture misalignment and scale errors:

$$
\pmb {\omega} (t) = \pmb {T} _ {g}   \pmb {\omega} _ {b} ^ {b} (t) + \mathbf {b} ^ {g} (t) + \pmb {\eta} ^ {g} (t),\tag{11.4}
$$

where $\mathbf { \delta } _ { T _ { g } }$ is the shape matrix that models both misalignment and scale errors in the gyroscope measurements. Gyroscope measurements are often influenced by acceleration, a phenomenon called g-sensitivity. The magnitude of this influence is considered negligible if it is within the range of the additive white noise $\eta ^ { g } ( t )$ , while in some MEMS hardware, it can be more significant and modeled as follows:

$$
\pmb {\omega} (t) = \pmb {T} _ {g} \pmb {\omega} _ {b} ^ {b} (t) + \pmb {T} _ {s} \pmb {R} _ {w} ^ {b} (t) (\mathbf {a} ^ {w} (t) - \mathbf {g} ^ {w}) + \mathbf {b} ^ {g} (t) + \pmb {\eta} ^ {g} (t),\tag{11.5}
$$

where $\pmb { T _ { s } }$ is the g-sensitivity matrix, which can be estimated during calibration.

## 11.1.2 Initial Alignment

In SLAM it is customary to set the global coordinate frame to be the starting pose of the trajectory, $i . e . ,$ set the initial pose $\{ R _ { b } ^ { w } ( 0 ) , p ^ { w } ( 0 ) \}$ to be the identity pose. However, in INS, as the IMU measurements involve the gravitational force $\left( c . f . \ ( 1 1 . 1 ) \right)$ , we typically choose the world frame to be gravity-aligned, thus requiring to align the initial pose with the gravity direction. In other words, since the IMU measurements depend on the gravity direction, the orientation of the robot is no longer an arbitrary choice, and it must be consistent with the gravity direction. Specifically, we need to compute the rotation $R _ { b } ^ { w } ( 0 )$ that aligns the body (IMU) frame to the world frame. For simplicity, assume the robot is initially static, i.e., at the beginning of deployment, no specific force is applied to the robot and the commonly-used low-cost MEMS IMU only measures the gravitational force. Clearly, given only the local gravity measurement $\mathbf { g } ^ { b }$ , we cannot recover the rotation along gravity (i.e., yaw), which is thus up to free choice depending on the application. However, we can determine the rotation corresponding to roll and pitch via the following static initialization:

$$
\left\{ \begin{array}{l} \boldsymbol {z} _ {w} ^ {b} = \frac {\mathbf {g} ^ {b}}{| | \mathbf {g} ^ {b} | |} \\ \boldsymbol {x} _ {w} ^ {b} = \frac {\mathbf {e} _ {1} - \boldsymbol {z} _ {w} ^ {b} \mathbf {e} _ {1} ^ {\top} \boldsymbol {z} _ {w} ^ {b}}{| | \mathbf {e} _ {1} - \boldsymbol {z} _ {w} ^ {b} \mathbf {e} _ {1} ^ {\top} \boldsymbol {z} _ {w} ^ {b} | |} \\ \boldsymbol {y} _ {w} ^ {b} = \boldsymbol {z} _ {w} ^ {b} \times \boldsymbol {x} _ {w} ^ {b} \end{array} \right. \Rightarrow \boldsymbol {R} _ {w} ^ {b} = \left[ \begin{array}{c c c} \boldsymbol {x} _ {w} ^ {b} & \boldsymbol {y} _ {w} ^ {b} & \boldsymbol {z} _ {w} ^ {b} \end{array} \right]\tag{11.6}
$$

where we perform the Gram–Schmidt orthonormalization given vectors ${ \bf e } _ { 1 } = [ 1 0 0 ] ^ { \top }$ and $\mathbf { g } ^ { b } .$ , and $\times$ is the cross product. Intuitively, the last column of the rotation matrix $\mathbf { \mathcal { R } } _ { w } ^ { b }$ , namely $ { \boldsymbol { z } } _ { w } ^ { b }$ , is the direction of the z-axis of the world frame with respect to the body frame. Since the z-axis of the world frame is aligned with gravity, eq. (11.6) computes $z _ { w } ^ { b }$ from the measurement of the body-frame gravity vector $\mathbf { g } ^ { b }$ Then, the orthonormalization procedure computes orthonormal vectors $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { w } ^ { b } } ^ { b }$ and $\boldsymbol { y } _ { w } ^ { b }$ to complete the columns of the rotation matrix $\mathbf { \mathcal { R } } _ { w } ^ { b }$ for an arbitrary choice of yaw.

Alignment with High-end IMUs. When using high-end IMUs, the gyroscope is sensitive enough to measure the Earth rotation rate $\omega _ { i e }$ . In this case, assuming the chosen world frame is an inertial frame (e.g., the Earth-Centered Inertial frame, or ECI [318]), one can use the measurement of the body-frame gravity vector $\mathbf { g } ^ { b }$ and the Earth rotation rate $\omega _ { i e }$ to perform analytical alignment:

$$
\left\{ \begin{array}{l} \mathbf {g} ^ {b} = \boldsymbol {R} _ {w} ^ {b} \mathbf {g} ^ {w} \\ \boldsymbol {\omega} _ {i e} ^ {b} = \boldsymbol {R} _ {w} ^ {b} \boldsymbol {\omega} _ {i e} ^ {w} \\ \mathbf {g} ^ {b} \times \boldsymbol {\omega} _ {i e} ^ {b} = \boldsymbol {R} _ {w} ^ {b} \left(\mathbf {g} ^ {w} \times \boldsymbol {\omega} _ {i e} ^ {w}\right) \end{array} \Rightarrow \boldsymbol {R} _ {b} ^ {w} = \left[ \begin{array}{c} \mathbf {g} ^ {w ^ {\top}} \\ \boldsymbol {\omega} _ {i e} ^ {w ^ {\top}} \\ (\mathbf {g} ^ {w} \times \boldsymbol {\omega} _ {i e} ^ {w}) ^ {\top} \end{array} \right] ^ {- 1} \left[ \begin{array}{c} \mathbf {g} ^ {b ^ {\top}} \\ \boldsymbol {\omega} _ {i e} ^ {b ^ {\top}} \\ (\mathbf {g} ^ {b} \times \boldsymbol {\omega} _ {i e} ^ {w}) ^ {\top} \end{array} \right] \right.\tag{11.7}
$$

where the resulting rotation matrix $R _ { b } ^ { w }$ typically needs to be projected onto $\mathrm { S O ( 3 ) }$ to mitigate the impact of measurement noise.

## 11.2 IMU Preintegration and Factor Graphs

In the previous section, we have introduced the IMU measurement model (11.1)- (11.2), which relates the IMU measurements to the state of the robot, and in particular its pose and velocity, as well as the sensor biases. While in principle we can use these models to derive a Maximum a Posteriori estimator as described in Chapter 1, this leads to impractically large factor graphs: a typical IMU provides measurements at a high-rate (e.g., 200-1000 Hz) and the measurement model would require adding states to the factor graph at each IMU sampling time. The resulting factor graph would quickly become impractical to solve. The more astute reader might observe that a continuous-time formulation of the problem could circumvent the high-rate addition of variables to the factor graph. However, in a continuous-time formulation of inertial navigation, one would still need to add factors at high-rate, one for each measurement, again leading to an unwieldy factor graph.

In this chapter, we present the key idea of IMU preintegration, which provides a way to avoid adding states or measurements at IMU rate to the factor graph. The basic idea is to integrate IMU measurements over time to obtain relative motion measurements, such that these (fewer) motion measurements can be added to the factor graph instead. However, a naive integration of the IMU measurements (reviewed in Section 11.2.1) would still require repeating the integration of the measurements at each iteration of the factor graph solver (due to potential changes in the initial conditions for integration). IMU preintegration avoids this issue by separating terms that depend on the state variables from the measurements. The original idea of preintegration goes back to [709] and has been extended to operate on manifold in [334, 335]; in Section 11.2.2, we closely follow the presentation in [334, 335]; then discuss more advanced preintegration techniques in Section 11.2.3. As usual, we postpone the discussion of recent works on the topic to Section 11.5.

## 11.2.1 Motion Integration

In this section, we start by inferring the motion of the robot from IMU measurements. For this purpose we introduce the following kinematic model [790, 318]:

$$
\dot {\boldsymbol {R}} _ {b} ^ {w} = \boldsymbol {R} _ {b} ^ {w} (\boldsymbol {\omega} _ {b} ^ {b}) ^ {\wedge}, \qquad \dot {\boldsymbol {v}} ^ {w} = \mathbf {a} ^ {w}, \qquad \dot {\boldsymbol {p}} ^ {w} = \boldsymbol {v} ^ {w},\tag{11.8}
$$

which describes the evolution of the rotation $R _ { b } ^ { w }$ , translation $\pmb { p } ^ { w }$ , and velocity $\pmb { v } ^ { w }$ of the body frame $\mathcal { F } ^ { b }$ with respect to the world frame ${ \mathcal { F } } ^ { w }$

The state at time $t + \Delta t ,$ where $\Delta t$ is the IMU sampling period, is obtained by

integrating (11.8):

$$
\begin{array}{l} \pmb {R} _ {b} ^ {w} (t + \Delta t) = \pmb {R} _ {b} ^ {w} (t) \mathrm{Exp} \left(\int_ {t} ^ {t + \Delta t} \pmb {\omega} _ {b} ^ {b} (\tau) d \tau\right) \\ \pmb {v} ^ {w} (t + \Delta t) = \pmb {v} ^ {w} (t) + \int_ {t} ^ {t + \Delta t} \pmb {\mathbf {a}} ^ {w} (\tau) d \tau \\ \pmb {p} ^ {w} (t + \Delta t) = \pmb {p} ^ {w} (t) + \int_ {t} ^ {t + \Delta t} \pmb {v} ^ {w} (\tau) d \tau \end{array}\tag{11.9}
$$

(11.10)

where in the first equation we assumed that the direction of the angular velocity $\omega _ { b } ^ { b }$ does not change in the interval $[ t , t + \Delta t ] . ^ { 4 }$ Further assuming that $\mathbf { a } ^ { w }$ and $\omega _ { b } ^ { b }$ remain constant in the time interval $[ t , t + \Delta t ]$ , we can write:

$$
\begin{array}{r l} & {\pmb {R} _ {b} ^ {w} (t + \Delta t) = \pmb {R} _ {b} ^ {w} (t) \mathrm{Exp} (\pmb {\omega} _ {b} ^ {b} (t) \Delta t)} \\ & {\pmb {v} ^ {w} (t + \Delta t) = \pmb {v} ^ {w} (t) + \pmb {a} ^ {w} (t) \Delta t} \\ & {\pmb {p} ^ {w} (t + \Delta t) = \pmb {p} ^ {w} (t) + \pmb {v} ^ {w} (t) \Delta t + \frac {1}{2} \pmb {a} ^ {w} (t) \Delta t ^ {2}.} \end{array}\tag{11.11}
$$

More generally, eq. (11.11) can be understood as applying Euler integration to numerically solve the integrals in (11.9). Using Eqs. (11.1)-(11.2), we can write $\mathbf { a } ^ { w }$ and $\omega _ { b } ^ { b }$ as functions of the IMU measurements, hence (11.11) becomes

$$
\begin{array}{r l} \boldsymbol {R} (t + \Delta t) = & \boldsymbol {R} (t) \mathrm{Exp} \left(\left(\tilde {\boldsymbol {\omega}} (t) - \mathbf {b} ^ {g} (t) - \boldsymbol {\eta} ^ {g d} (t)\right) \Delta t\right) \\ \boldsymbol {v} (t + \Delta t) = & \boldsymbol {v} (t) + \mathbf {g} \Delta t + \boldsymbol {R} (t) \left(\tilde {\mathbf {a}} (t) - \mathbf {b} ^ {a} (t) - \boldsymbol {\eta} ^ {a d} (t)\right) \Delta t \\ \boldsymbol {p} (t + \Delta t) = & \boldsymbol {p} (t) + \boldsymbol {v} (t) \Delta t + \frac {1}{2} \mathbf {g} \Delta t ^ {2} + \frac {1}{2} \boldsymbol {R} (t) \left(\tilde {\mathbf {a}} (t) - \mathbf {b} ^ {a} (t) - \boldsymbol {\eta} ^ {a d} (t)\right) \Delta t ^ {2}, \end{array}\tag{11.12}
$$

where we dropped the coordinate frame subscripts for readability (the notation should be unambiguous from now on). This numeric integration of the velocity and position assumes a constant orientation $\mathbf { } R ( t )$ for the time of integration between two measurements, which is not an exact solution of the diferential equation (11.8) for measurements with non-zero rotation rate. In practice, the use of a high-rate IMU mitigates the efects of this approximation. We adopt the integration scheme (11.12) as it is simple and amenable for modeling and uncertainty propagation, and then discuss more advanced integration techniques in Section 11.2.3. The covariance of the discrete-time noise $\eta ^ { g d }$ is a function of the sampling rate and relates to the continuous-time noise $\eta ^ { g }$ via $\begin{array} { r } { \mathrm { C o v } ( \pmb { \eta } ^ { g d } ( t ) ) = \frac { 1 } { \Delta t } \mathrm { C o v } ( \pmb { \eta } ^ { g } ( t ) ) , } \end{array}$ . The same relation holds for $\eta ^ { a d }$ (cf., [232, Appendix]).

While Eq. (11.12) could be readily seen as a probabilistic constraint in a factor graph, it would require including states in the factor graph at high rate. Intuitively, Eq. (11.12) relates states at time t and $t + \Delta t$ , where $\Delta t$ is the sampling period of the IMU, hence we would have to add new states in the estimation at every new IMU measurement [503].

![](images/0db1d82356b4755dcd74b313d9edfb3c18c45aa07f0bb0d9e14e8391561de54c.jpg)  
Figure 11.1 Diferent rates for IMU and camera. From [335] (©2016 IEEE).

We can try to avoid this issue by integrating over longer time intervals. In particular, if we assume that we already have a factor graph modeling other sensor measurements in our problem $( e . g .$ , the vision measurements in Chapter 7), we can use the expression (11.12) and integrate IMU measurements between two temporally consecutive states in our factor graph. We are going to refer to these states as “keyframe states”.<sup>5</sup> Iterating the IMU integration (11.12) for all $\Delta t$ intervals between two consecutive keyframes at times $t _ { i }$ and $t _ { j } \ \left( c . f . , \ \mathrm { F i g . \ 1 1 . 1 } \right)$ , we get:<sup>6</sup>

$$
\begin{array}{l} \boldsymbol {R} _ {j} = \boldsymbol {R} _ {i} \prod_ {k = i} ^ {j - 1} \mathrm{Exp} \left(\left(\tilde {\omega} _ {k} - \mathbf {b} _ {k} ^ {g} - \boldsymbol {\eta} _ {k} ^ {g d}\right) \Delta t\right), \\ \boldsymbol {v} _ {j} = \boldsymbol {v} _ {i} + \mathbf {g} \Delta t _ {i j} + \sum_ {k = i} ^ {j - 1} \boldsymbol {R} _ {k} \Big (\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {k} ^ {a} - \boldsymbol {\eta} _ {k} ^ {a d} \Big) \Delta t \\ \boldsymbol {p} _ {j} = \boldsymbol {p} _ {i} + \sum_ {k = i} ^ {j - 1} \Big [ \boldsymbol {v} _ {k} \Delta t + \frac {1}{2} \mathbf {g} \Delta t ^ {2} + \frac {1}{2} \boldsymbol {R} _ {k} \Big (\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {k} ^ {a} - \boldsymbol {\eta} _ {k} ^ {a d} \Big) \Delta t ^ {2} \Big ] \end{array}\tag{11.13}
$$

where we introduced the shorthands $\begin{array} { r } { \Delta t { _ { i j } } \doteq \sum _ { k = i } ^ { j - 1 } \Delta t } \end{array}$ and $( \cdot ) _ { i } \doteq ( \cdot ) ( t _ { i } )$ for readability. While Eq. (11.13) already provides an estimate of the motion between time $t _ { i }$ and $t _ { j } .$ it has the drawback that the integration in (11.13) has to be repeated whenever the linearization point at time $t _ { i }$ changes [645] $( e . g .$ , at each iteration of a Gauss-Newton solver). For instance, a change in the rotation $\mathbf { { \mathit { R } } } _ { i }$ implies a change in all future rotations $R _ { k } , \ k = i , \ldots , j - 1$ , and makes necessary to re-evaluate summations and products in (11.13).

## 11.2.2 IMU Preintegration on Manifold

Here we show that a small manipulation of the motion integration results (11.13) allows computing relative measurements between states at time $t _ { i }$ and $t _ { j }$ that do not need to be recomputed when the linearization point changes. The key insight is to express measurements in a local frame (such that they do not change when the global state estimate of the robot changes) and isolating the contribution of gravity (which again carries information about the global frame). This process leads to computing the so called preintegrated IMU measurements, which constrain the motion between consecutive states in the factor graph.

Towards this goal, we rearrange the terms in (11.13) and define the following relative motion increments that are independent of the pose and velocity at $t _ { i }$ :

$$
\begin{array}{l} \Delta \boldsymbol {R} _ {i j} \doteq \boldsymbol {R} _ {i} ^ {\mathsf {T}} \boldsymbol {R} _ {j} = \prod_ {k = i} ^ {j - 1} \mathrm{Exp} \left(\left(\tilde {\omega} _ {k} - \mathbf {b} _ {k} ^ {g} - \boldsymbol {\eta} _ {k} ^ {g d}\right) \Delta t\right) \\ \Delta \boldsymbol {v} _ {i j} \doteq \boldsymbol {R} _ {i} ^ {\mathsf {T}} (\boldsymbol {v} _ {j} - \boldsymbol {v} _ {i} - \mathbf {g} \Delta t _ {i j}) = \sum_ {k = i} ^ {j - 1} \Delta \boldsymbol {R} _ {i k} \big (\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {k} ^ {a} - \boldsymbol {\eta} _ {k} ^ {a d} \big) \Delta t \\ \Delta \boldsymbol {p} _ {i j} \doteq \boldsymbol {R} _ {i} ^ {\mathsf {T}} (\boldsymbol {p} _ {j} - \boldsymbol {p} _ {i} - \boldsymbol {v} _ {i} \Delta t _ {i j} - \frac {1}{2} \mathbf {g} \Delta t _ {i j} ^ {2}) \\ = \sum_ {k = i} ^ {j - 1} \left[ \Delta \boldsymbol {v} _ {i k} \Delta t + \frac {1}{2} \Delta \boldsymbol {R} _ {i k} (\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {k} ^ {a} - \boldsymbol {\eta} _ {k} ^ {a d}) \Delta t ^ {2} \right], \end{array}\tag{11.14}
$$

where $\Delta R _ { i k } \ \doteq \ R _ { i } ^ { \top } R _ { k }$ and $\Delta \pmb { v } _ { i k } \ \doteq \ R _ { i } ^ { \top } \left( \pmb { v } _ { k } - \pmb { v } _ { i } - \mathbf { g } \Delta t _ { i k } \right)$ . We highlight that, in contrast to the “delta” rotation $\Delta R _ { i j }$ , neither $\Delta { } v _ { i j }$ nor $\Delta { p } _ { i j }$ correspond to the true physical change in velocity and position but are defined in a way that make the righthand side of (11.14) independent from the state at time i as well as gravitational efects. Indeed, we will be able to compute the right-hand side of (11.14) directly from the inertial measurements between the two keyframes.

Unfortunately, summations and products in (11.14) are still function of the bias estimate. We tackle this problem in two steps. In Section 11.2.2.1, we assume $\mathbf { b } _ { i }$ is known; then, in Section 11.2.2.3 we show how to avoid repeating the integration when the bias estimate changes. In both cases, we assume the biases remain constant between times $t _ { i }$ and $t _ { j } { \mathrm { : } }$ :

$$
\mathbf {b} _ {i} ^ {g} = \mathbf {b} _ {i + 1} ^ {g} = \ldots = \mathbf {b} _ {j - 1} ^ {g}, \quad \mathbf {b} _ {i} ^ {a} = \mathbf {b} _ {i + 1} ^ {a} = \ldots = \mathbf {b} _ {j - 1} ^ {a}.\tag{11.15}
$$

## 11.2.2.1 Preintegrated IMU Measurements

Equation (11.14) relates the states of keyframes i and j (left-hand side) to the measurements (right-hand side). In this sense, it can be already understood as a measurement model. Unfortunately, it has a fairly intricate dependence on the measurement noise and this complicates a direct application of MAP estimation; intuitively, the MAP estimator requires to clearly define the densities (and their loglikelihood) of the measurements. In this section we manipulate (11.14) so to make easier the derivation of the measurement log-likelihood. More concretely, we isolate the noise terms of the individual inertial measurements in (11.14). As discussed above, within this section assume that the bias at time $t _ { i }$ is known.

Let us start with the rotation increment $\Delta R _ { i j }$ in (11.14). Towards this goal, we use the following properties of the exponential map for SO(3) (cf. Chapter 2):

$$
\mathrm{Exp} (\phi + \delta \phi) \approx \mathrm{Exp} (\phi) \mathrm{Exp} (\mathsf {J} _ {r} (\phi) \delta \phi),\tag{11.16}
$$

$$
\mathrm{Exp} (\phi) \boldsymbol {R} = \boldsymbol {R} \mathrm{Exp} (\boldsymbol {R} ^ {\mathsf {T}} \phi).\tag{11.17}
$$

where the first relations is a first-order approximation of the exponential of a sum, and the second can be derived from the group’s adjoint representation.

Using (11.16) and (11.17), we rearrange the terms in the expression of $\Delta R _ { i j }$ in (11.14), by “moving” the noise to the end:

$$
\begin{array}{l} \Delta \boldsymbol {R} _ {i j} \stackrel {{\text {eq. (11.16)}}} {{\simeq}} \prod_ {k = i} ^ {j - 1} \left[ \operatorname{Exp} \left((\tilde {\boldsymbol {\omega}} _ {k} - \mathbf {b} _ {i} ^ {g})   \Delta t\right) \operatorname{Exp} \left(- \mathrm{J} _ {r} ^ {k}   \boldsymbol {\eta} _ {k} ^ {g d}   \Delta t\right) \right] \\ \stackrel {{\text {eq. (11.17)}}} {{=}} \Delta \tilde {\boldsymbol {R}} _ {i j} \prod_ {k = i} ^ {j - 1} \operatorname{Exp} \left(- \Delta \tilde {\boldsymbol {R}} _ {k + 1 j} ^ {\mathsf {T}}   \mathrm{J} _ {r} ^ {k}   \boldsymbol {\eta} _ {k} ^ {g d}   \Delta t\right) \\ \doteq \Delta \tilde {\boldsymbol {R}} _ {i j} \operatorname{Exp} (- \delta \phi_ {i j}) \end{array}\tag{11.18}
$$

with $\mathsf { J } _ { r } ^ { k } \doteq \mathsf { J } _ { r } ^ { k } \bigl ( ( \tilde { \omega } _ { k } - \mathbf { b } _ { i } ^ { g } ) \Delta t \bigr )$ . In the last line of (11.18), we defined the preintegrated rotation measurement $\Delta \tilde { R } _ { i j } \doteq \Pi _ { k = i } ^ { j - 1 }$ Exp $( ( \tilde { \omega } _ { k } - \mathbf { b } _ { i } ^ { g } ) \Delta t )$ , and its noise $\delta \phi _ { i j }$ , which will be further analysed in the next section.

Similarly, we can manipulate the velocity and position equations in (11.14) by using the following relations:

$$
\exp (\phi^ {\wedge}) \approx \mathbf {I} + \phi^ {\wedge},\tag{11.19}
$$

$$
\mathbf {a} ^ {\wedge} \mathbf {b} = - \mathbf {b} ^ {\wedge} \mathbf {a}, \quad \forall \mathbf {a}, \mathbf {b} \in \mathbb {R} ^ {3},\tag{11.20}
$$

where the first relation is a first-order approximation of the exponential map at the origin, while the second is a property of the wedge operator of a vector.

Substituting (11.18) back into the expression of $\Delta { \boldsymbol v _ { i j } }$ in (11.14), using the firstorder approximation (11.19) for $\mathrm { E x p } \left( - \delta \phi _ { i j } \right)$ , and dropping higher-order noise terms, we obtain:

$$
\begin{array}{l} \Delta \boldsymbol {v} _ {i j} \stackrel {{\text {eq. (11.19)}}} {{\simeq}} \sum_ {k = i} ^ {j - 1} \Delta \tilde {\boldsymbol {R}} _ {i k} (\mathbf {I} - \delta \phi_ {i k} ^ {\wedge}) \left(\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {i} ^ {a}\right) \Delta t - \Delta \tilde {\boldsymbol {R}} _ {i k} \boldsymbol {\eta} _ {k} ^ {a d} \Delta t \\ \stackrel {{\text {eq. (11.20)}}} {{=}} \Delta \tilde {\boldsymbol {v}} _ {i j} + \sum_ {k = i} ^ {j - 1} \left[ \Delta \tilde {\boldsymbol {R}} _ {i k} \left(\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {i} ^ {a}\right) ^ {\wedge} \delta \phi_ {i k} \Delta t - \Delta \tilde {\boldsymbol {R}} _ {i k} \boldsymbol {\eta} _ {k} ^ {a d} \Delta t \right] \\ \doteq \Delta \tilde {\boldsymbol {v}} _ {i j} - \delta \boldsymbol {v} _ {i j} \end{array}\tag{11.21}
$$

where we defined the preintegrated velocity measurement $\begin{array} { r } { \Delta \tilde { \mathbf { v } } _ { i j } \doteq \sum _ { k = i } ^ { j - 1 } \Delta \tilde { R } _ { i k } ( \tilde { \mathbf { a } } _ { k } - \mathbf { b } _ { i } ^ { a } ) \Delta t } \end{array}$ and its noise $\delta \pmb { v } _ { i j }$

Similarly, substituting (11.18) and (11.21) in the expression of $\Delta { { p } _ { i j } }$ in (11.14), and using the first-order approximation (11.19), we obtain:

$$
\begin{array}{l} \Delta \boldsymbol {p} _ {i j} \stackrel {{\text { eq. (11.19)}}} {{\simeq}} \sum_ {k = i} ^ {j - 1} \Big [ (\Delta \tilde {\boldsymbol {v}} _ {i k} - \delta \boldsymbol {v} _ {i k}) \Delta t + \frac {1}{2} \Delta \tilde {\boldsymbol {R}} _ {i k} (\mathbf {I} - \delta \phi_ {i k} ^ {\wedge})   (\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {i} ^ {a})   \Delta t ^ {2} \\ \qquad - \frac {1}{2} \Delta \tilde {\boldsymbol {R}} _ {i k} \boldsymbol {\eta} _ {k} ^ {a d}   \Delta t ^ {2} \Big ] \\ \stackrel {{\text { eq. (11.20)}}} {{=}} \Delta \tilde {\boldsymbol {p}} _ {i j} + \sum_ {k = i} ^ {j - 1} \Big [ - \delta \boldsymbol {v} _ {i k} \Delta t + \frac {1}{2} \Delta \tilde {\boldsymbol {R}} _ {i k}   (\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {i} ^ {a}) ^ {\wedge}   \delta \phi_ {i k} \Delta t ^ {2} \\ \qquad - \frac {1}{2} \Delta \tilde {\boldsymbol {R}} _ {i k} \boldsymbol {\eta} _ {k} ^ {a d}   \Delta t ^ {2} \Big ] \\ \doteq \Delta \tilde {\boldsymbol {p}} _ {i j} - \delta \boldsymbol {p} _ {i j}, \end{array}\tag{11.22}
$$

where we defined the preintegrated position measurement $\Delta \tilde { p } _ { i j }$ and its noise $\delta \pmb { p } _ { i j }$ Substituting the expressions (11.18), (11.21), (11.22) back in the original definition of $\Delta R _ { i j } , \Delta \pmb { v } _ { i j } , \Delta \pmb { p } _ { i j }$ in (11.14), we finally get our preintegrated measurement model (remember Exp $\left( - \delta \phi _ { i j } \right) ^ { \mathsf { T } } = \mathrm { E x p } \left( \delta \phi _ { i j } \right) )$

$$
\begin{array}{l} \Delta \tilde {\boldsymbol {R}} _ {i j} = \boldsymbol {R} _ {i} ^ {\mathsf {T}} \boldsymbol {R} _ {j} \mathrm{Exp} (\delta \phi_ {i j}) \\ \Delta \tilde {\boldsymbol {v}} _ {i j} = \boldsymbol {R} _ {i} ^ {\mathsf {T}} (\boldsymbol {v} _ {j} - \boldsymbol {v} _ {i} - \mathbf {g} \Delta t _ {i j}) + \delta \boldsymbol {v} _ {i j} \\ \Delta \tilde {\boldsymbol {p}} _ {i j} = \boldsymbol {R} _ {i} ^ {\mathsf {T}} (\boldsymbol {p} _ {j} - \boldsymbol {p} _ {i} - \boldsymbol {v} _ {i} \Delta t _ {i j} - \frac {1}{2} \mathbf {g} \Delta t _ {i j} ^ {2}) + \delta \boldsymbol {p} _ {i j} \end{array}\tag{11.23}
$$

where our compound measurements are written as a function of the (to-be-estimated) state “plus” a random noise, described by the random vector $[ \delta \phi _ { i j } ^ { \top } , \delta \pmb { v } _ { i j } ^ { \top } , \delta \pmb { p } _ { i j } ^ { \top } ] ^ { \top }$

In summary, in this section we manipulated the measurement model $( 1 1 . 1 4 )$ and rewrote it as (11.23). The advantage of the measurements in eq. (11.23) is that, for a suitable distribution of the noise, they can be used directly to instantiate factors between states at time $t _ { i }$ and $t _ { j }$ in our factor graph. The nature of the noise terms is discussed in the following section.

## 11.2.2.2 Noise Propagation

In this section we derive the statistics of the noise vector $[ \delta \phi _ { i j } ^ { \mathsf { T } } , \delta \pmb { v } _ { i j } ^ { \mathsf { T } } , \delta \pmb { p } _ { i j } ^ { \mathsf { T } } ] ^ { \mathsf { T } }$ . While we already observed that it is convenient to approximate the noise vector to be zeromean Normally distributed, it is of paramount importance to accurately model the noise covariance. In this section, we therefore provide a derivation of the covariance $\Sigma _ { i j }$ of the preintegrated measurements:

$$
\boldsymbol {\eta} _ {i j} ^ {\Delta} \doteq [ \delta \boldsymbol {\phi} _ {i j} ^ {\mathsf {T}}, \delta \boldsymbol {v} _ {i j} ^ {\mathsf {T}}, \delta \boldsymbol {p} _ {i j} ^ {\mathsf {T}} ] ^ {\mathsf {T}} \sim \mathcal {N} (\mathbf {0} _ {9 \times 1}, \boldsymbol {\Sigma} _ {i j}).\tag{11.24}
$$

We first consider the preintegrated rotation noise $\delta \phi _ { i j }$ . Recall from (11.18) that

$$
\operatorname{Exp} \left(- \delta \phi_ {i j}\right) \doteq \prod_ {k = i} ^ {j - 1} \operatorname{Exp} \left(- \Delta \tilde {\boldsymbol {R}} _ {k + 1 j} ^ {\intercal} \mathbb {J} _ {r} ^ {k} \boldsymbol {\eta} _ {k} ^ {g d} \Delta t\right).\tag{11.25}
$$

Taking the Log on both sides and changing signs, we get:

$$
\delta \phi_ {i j} = - \mathrm{Log} \left(\prod_ {k = i} ^ {j - 1} \mathrm{Exp} \left(- \Delta \tilde {\boldsymbol {R}} _ {k + 1 j} ^ {\mathsf {T}} \mathbb {J} _ {r} ^ {k}   \boldsymbol {\eta} _ {k} ^ {g d}   \Delta t\right)\right).\tag{11.26}
$$

Next, we use the following first-order approximation holds for SO(3) logarithm:

$$
\mathrm{Log} \big (\mathrm{Exp} (\phi) \mathrm{Exp} (\delta \phi) \big) \approx \phi + \mathrm{J} _ {r} ^ {- 1} (\phi) \delta \phi .\tag{11.27}
$$

where $\ J _ { r } ^ { - 1 } ( \phi )$ is the inverse of the right Jacobian. Repeated application of (11.27) (recall that $\eta _ { k } ^ { g d }$ as well as $\delta \phi _ { i j }$ are small rotation noises, hence the right Jacobians are close to the identity) produces:

$$
\delta \phi_ {i j} \simeq \sum_ {k = i} ^ {j - 1} \Delta \tilde {\boldsymbol {R}} _ {k + 1 j} ^ {\intercal} \mathsf {J} _ {r} ^ {k} \boldsymbol {\eta} _ {k} ^ {g d} \Delta t\tag{11.28}
$$

Up to first order, the noise $\delta \phi _ { i j }$ is zero-mean and Gaussian, as it is a linear combination of zero-mean noise terms $\eta _ { k } ^ { g d }$

Dealing with the noise terms $\delta \pmb { v } _ { i j }$ and $\delta \pmb { p } _ { i j }$ is now easy: these are linear combinations of the acceleration noise $\eta _ { k } ^ { a d }$ and the preintegrated rotation noise $\delta \phi _ { i j }$ , hence they are also zero-mean and Gaussian. Simple manipulation leads to:

$$
\delta \boldsymbol {v} _ {i j} \simeq \sum_ {k = i} ^ {j - 1} \left[ - \Delta \tilde {\boldsymbol {R}} _ {i k} \left(\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {i} ^ {a}\right) ^ {\wedge} \delta \phi_ {i k} \Delta t + \Delta \tilde {\boldsymbol {R}} _ {i k} \boldsymbol {\eta} _ {k} ^ {a d} \Delta t \right]\tag{11.29}
$$

$$
\delta \pmb {p} _ {i j} \simeq \sum_ {k = i} ^ {j - 1} \left[ \delta \pmb {v} _ {i k} \Delta t - \frac {1}{2} \Delta \tilde {\pmb {R}} _ {i k} (\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {i} ^ {a}) ^ {\wedge} \delta \phi_ {i k} \Delta t ^ {2} + \frac {1}{2} \Delta \tilde {\pmb {R}} _ {i k} \pmb {\eta} _ {k} ^ {a d} \Delta t ^ {2} \right]
$$

where the relations are valid up to the first order.

Eqs. (11.28)-(11.29) express the preintegrated noise $\eta _ { i j } ^ { \Delta }$ as a linear function of the IMU measurement noise $\pmb { \eta } _ { k } ^ { d } \doteq [ \pmb { \eta } _ { k } ^ { g d } , \pmb { \eta } _ { k } ^ { a d } ] , k = 1 , \ldots , \bar { j } - 1$ . Therefore, from the knowledge of the covariance of $\eta _ { k } ^ { d }$ (given in the IMU specifications), we can compute the covariance of $\eta _ { i j } ^ { \Delta }$ , namely $\Sigma _ { i j }$ , by a simple linear propagation.

An extended derivation of the noise propagation can be found in [335], which also provides an iterative expression to compute the covariance by incrementally adding new measurements as they are collected. The iterative computation leads to simpler expressions and is more amenable for online inference.

## 11.2.2.3 Incorporating Bias Updates

In the previous section, we assumed that the bias, say $\{ \bar { \mathbf { b } } _ { i } ^ { a } , \bar { \mathbf { b } } _ { i } ^ { g } \}$ , that is used during preintegration between k = i and $k = j$ is correct and does not change. However, more likely, the bias estimate changes by a small amount δb during optimization. One solution would be to recompute the delta measurements when the bias changes; however, that is computationally expensive. Instead, given a bias update b $\bar { \mathbf { b } } + \delta \mathbf { b } .$ , we can update the delta measurements using a first-order expansion:

$$
\begin{array}{r} \Delta \tilde {\pmb {R}} _ {i j} (\mathbf {b} _ {i} ^ {g}) \simeq \Delta \tilde {\pmb {R}} _ {i j} (\bar {\mathbf {b}} _ {i} ^ {g}) \mathrm{Exp} \Big (\frac {\partial \Delta \bar {\pmb {R}} _ {i j}}{\partial \mathbf {b} ^ {g}} \delta \mathbf {b} ^ {g} \Big) \\ \Delta \tilde {\pmb {v}} _ {i j} (\mathbf {b} _ {i} ^ {g}, \mathbf {b} _ {i} ^ {a}) \simeq \Delta \tilde {\pmb {v}} _ {i j} (\bar {\mathbf {b}} _ {i} ^ {g}, \bar {\mathbf {b}} _ {i} ^ {a}) + \frac {\partial \Delta \bar {\pmb {v}} _ {i j}}{\partial \mathbf {b} ^ {g}} \delta \mathbf {b} _ {i} ^ {g} + \frac {\partial \Delta \bar {\pmb {v}} _ {i j}}{\partial \mathbf {b} ^ {a}} \delta \mathbf {b} _ {i} ^ {a} \\ \Delta \tilde {\pmb {p}} _ {i j} (\mathbf {b} _ {i} ^ {g}, \mathbf {b} _ {i} ^ {a}) \simeq \Delta \tilde {\pmb {p}} _ {i j} (\bar {\mathbf {b}} _ {i} ^ {g}, \bar {\mathbf {b}} _ {i} ^ {a}) + \frac {\partial \Delta \bar {\pmb {p}} _ {i j}}{\partial \mathbf {b} ^ {g}} \delta \mathbf {b} _ {i} ^ {g} + \frac {\partial \Delta \bar {\pmb {p}} _ {i j}}{\partial \mathbf {b} ^ {a}} \delta \mathbf {b} _ {i} ^ {a} \end{array}\tag{11.30}
$$

This is similar to the bias correction in [709] but operates directly on $\mathrm { S O ( 3 ) }$ . The Jacobians $\big \{ \frac { \partial \Delta \bar { R } _ { i j } } { \partial { \bf b } ^ { g } } , \frac { \partial \Delta \bar { \bf v } _ { i j } } { \partial { \bf b } ^ { g } } , \underline { { \bf \dots } } . \big \}$ (computed at $\bar { \mathbf { b } } _ { i }$ , the bias estimate at integration time) describe how the measurements change due to a change in the bias estimate. The Jacobians remain constant and can be precomputed during the preintegration. The derivation of the Jacobians is very similar to the one we used in Section 11.2.2.1 to express the measurements as a large value plus a small perturbation and is given in [335].

## 11.2.2.4 Preintegrated IMU Factors and Bias Models

Given the preintegrated measurement model in (11.23) and since measurement noise is zero-mean and Gaussian (with covariance $\pmb { \Sigma } _ { i j } )$ up to first order (11.24), it is now easy to write the residual errors $r _ { \mathcal { T } _ { i j } } \doteq [ r _ { \Delta R _ { i j } } ^ { \mathsf { T } } , r _ { \Delta v _ { i j } } ^ { \mathsf { T } } , r _ { \Delta p _ { i j } } ^ { \mathsf { T } } ] ^ { \mathsf { T } } \in \mathbb { R } ^ { 9 }$ , which will appear in the factor graph optimization:

$$
\begin{array}{r l} & {\pmb {r} _ {\Delta \pmb {R} _ {i j}} \doteq \mathrm{Log} \left(\left(\Delta \tilde {\pmb {R}} _ {i j} (\bar {\mathbf {b}} _ {i} ^ {g}) \mathrm{Exp} \left(\frac {\partial \Delta \bar {\pmb {R}} _ {i j}}{\partial \mathbf {b} ^ {g}} \delta \mathbf {b} ^ {g}\right)\right) ^ {\top} \pmb {R} _ {i} ^ {\top} \pmb {R} _ {j}\right)} \\ & {\pmb {r} _ {\Delta \pmb {v} _ {i j}} \doteq \pmb {R} _ {i} ^ {\top} (\pmb {v} _ {j} - \pmb {v} _ {i} - \mathbf {g} \Delta t _ {i j})} \\ & {\qquad - \left[ \Delta \tilde {\pmb {v}} _ {i j} (\bar {\mathbf {b}} _ {i} ^ {g}, \bar {\mathbf {b}} _ {i} ^ {a}) + \frac {\partial \Delta \bar {\pmb {v}} _ {i j}}{\partial \mathbf {b} ^ {g}} \delta \mathbf {b} ^ {g} + \frac {\partial \Delta \bar {\pmb {v}} _ {i j}}{\partial \mathbf {b} ^ {a}} \delta \mathbf {b} ^ {a} \right]} \\ & {\pmb {r} _ {\Delta \pmb {p} _ {i j}} \doteq \pmb {R} _ {i} ^ {\top} (\pmb {p} _ {j} - \pmb {p} _ {i} - \pmb {v} _ {i} \Delta t _ {i j} - \frac 12 \mathbf {g} \Delta t _ {i j} ^ {2})} \\ & {\qquad - \left[ \Delta \tilde {\pmb {p}} _ {i j} (\bar {\mathbf {b}} _ {i} ^ {g}, \bar {\mathbf {b}} _ {i} ^ {a}) + \frac {\partial \Delta \bar {\pmb {p}} _ {i j}}{\partial \mathbf {b} ^ {g}} \delta \mathbf {b} ^ {g} + \frac {\partial \Delta \bar {\pmb {p}} _ {i j}}{\partial \mathbf {b} _ {a}} \delta \mathbf {b} ^ {a} \right],} \end{array}\tag{11.31}
$$

in which we also included the bias updates of Eq. (11.30). These terms can be readily added to the factor graph by adding the term $\left. \pmb { r } _ { \mathbb { Z } _ { i j } } \right. _ { \pmb { \Sigma } _ { i j } } ^ { 2 }$ to the objective of the minimization.

When presenting the IMU model (11.1)-(11.2), we said that biases are slowly time-varying quantities. Hence, we model them with a “Brownian motion”, i.e., integrated white noise:

$$
\dot {\mathbf {b}} ^ {g} (t) = \pmb {\eta} ^ {b g}, \qquad \dot {\mathbf {b}} ^ {a} (t) = \pmb {\eta} ^ {b a}.\tag{11.32}
$$

Integrating (11.32) over the time interval $[ t _ { i } , t _ { j } ]$ between two consecutive keyframes i and j we get:

$$
\mathbf {b} _ {j} ^ {g} = \mathbf {b} _ {i} ^ {g} + \boldsymbol {\eta} ^ {b g d}, \qquad \mathbf {b} _ {j} ^ {a} = \mathbf {b} _ {i} ^ {a} + \boldsymbol {\eta} ^ {b a d},\tag{11.33}
$$

where, as done before, we use the shorthand $\mathbf { b } _ { i } ^ { g } \doteq \mathbf { b } ^ { g } ( t _ { i } )$ , and we define the discrete noises $\eta ^ { b g d }$ and $\eta ^ { b a d }$ , which have zero mean and covariance $\Sigma ^ { b g d } \doteq \Delta t _ { i j } \mathrm { C o v } ( \pmb { \eta } ^ { b g } )$ and $\Sigma ^ { b a d } \doteq \Delta t _ { i j } \mathrm { C o v } ( \pmb { \eta } ^ { b a } )$ , respectively (cf. [232, Appendix]).

The model (11.33) can be readily included in our factor graph, as a further additive term in the objective function, for all consecutive keyframes:

$$
\left\| \boldsymbol {r} _ {\mathbf {b} _ {i j}} \right\| ^ {2} \doteq \left\| \mathbf {b} _ {j} ^ {g} - \mathbf {b} _ {i} ^ {g} \right\| _ {\boldsymbol {\Sigma} ^ {b g d}} ^ {2} + \left\| \mathbf {b} _ {j} ^ {a} - \mathbf {b} _ {i} ^ {a} \right\| _ {\boldsymbol {\Sigma} ^ {b a d}} ^ {2}\tag{11.34}
$$

## 11.2.3 Advanced Preintegration Techniques

In this section, we look at some limitations of the standard preintegration approach and explore newer alternatives. We first look at the underlying signal and motion assumptions embedded in (11.14). Then we go through various works that alleviate these assumptions leading to more accurate preintegrated measurements, thus improved localization and mapping accuracy when used for aided inertial navigation. Note that we do not detail the derivation of each of the methods and invite the reader to refer to the corresponding papers for a more extensive treatment.

## 11.2.3.1 Numerical Integration Accuracy

As described in the previous section, standard preintegration relies on the Euler method to integrate inertial signals into rotation, velocity, and position pseudomeasurements at discrete times. This approach is fast and eficient but introduces integration error (hence drift) in the preintegration process. In short, the Euler method consists in applying the rectangle rule to a signal to numerically obtain its integral. As illustrated in Figure 11.2 (left), it means that the signal is approximated with piecewise constant chunks sampled at a given frequency. In the context of inertial systems, the samples correspond to accelerometer or gyroscope measurements.

With a fairly low sampling frequency, the piecewise constant assumption does not accurately represent the input signal. Consequently, the double integration rapidly accumulates error (Figure 11.2 (top right)). A possible workaround consists in increasing the sampling frequency of the signal (Figure 11.2 (bottom)). However, in real-world inertial navigation problems, the sampling frequency is limited by the hardware characteristics of the inertial sensor.

In [631], the authors propose to use GP regression<sup>7</sup> as a mean to virtually upsample the input signal at any chosen timestamp for both the gyroscope and accelerometer data. While improving over standard preintegration, such an approach still performs numerical integration based on the piecewise constant assumption and does not fully leverage the continuous nature of GP models. Below, we review more sophisticated integration approaches.

Euler integration low sampling frequency (RMSE vel.: 0.0123 m/s

![](images/ce17a63b640959b908841e492f8bb182bae1e1375b7ebd467370ab2d47b4cbd6.jpg)

![](images/86b550ec8e6cc4edb9f3c09a12b1fd01baa6ca0951073aed2c46ef23d668d9b7.jpg)

![](images/b1244955a572871cf235a543d8beb8b0b540019c550c0381e3988057d425cba3.jpg)

Euler integration medium sampling frequency (RMSE vel.: 0.0024 m/s, pos.: 0.0023 m)  
![](images/c86a4fe04c0fad6b0675ca4ad2eadeb48f7e09f8c687c080aa6d2be94c8616df.jpg)

![](images/49db963c993b9525ba9ec1ce7850a2087ec784b9ba84929753bf7890616e44dc.jpg)

![](images/4be09b37823b8e3c5dd623da06d985ec209f995a5b7bb8b85044342ba18563a4.jpg)  
Figure 11.2 Example of Euler integration (with known initial conditions) using low (top row) and high (bottom row) sampling frequencies.

## 11.2.3.2 Continuous Acceleration Preintegration

Another way to reduced the integration error is to leverage continuous-time representations —which are not limited to discrete timestamps— that better approximate the true inertial signals and perform analytical integration. Atop the gain in accuracy, continuous-time representations allow for asynchronous query of the preintegrated measurements. This is especially useful when performing inertialaided state estimation with other sensors that are not hardware-synchronized or that have completely asynchronous sampling processes (e.g., event cameras).

A challenging component of preintegration is dealing with the space of rotations. The non-commutative nature of rotation operations prevents the use of numerous tools available for classic Riemann integration. Accordingly, several works have dissociated the rotational and translation parts of preintegration. In this subsection, we first explore the translation component of preintegration using continuous-time representations while assuming solved rotation integration. Continuous-time integration over the rotation space will be addressed in the following subsection.

In [300], after using a zeroth-order integrator [1105] to integrate the gyroscope measurements, the authors present a continuous formulation of the velocity and position preintegrated measurements by solving the continuous-time system of diferential equation (LTV) assuming constant accelerometer measurements or constant local acceleration (the two diferent models are presented in [300]). Compared with the standard preintegration [335] that considers constant global acceleration, the work [300] demonstrates that the assumption of constant local acceleration is more representative of real scenarios leading to an overall VIO accuracy improvement of around 5% on the EuRoc dataset [131] over both the standard preintegration and the constant accelerometer measurement model.

In order to loosen the constant acceleration motion model assumptions, one can approximate the input data with analytically integrable functions. Assuming that the rotational part of the preintegration is solved, the authors in [632] represent the rotation-corrected accelerometer measurements $\hat { \mathbf { a } } _ { k } .$ , defined as $\hat { \mathbf { a } } _ { k } = \Delta R _ { i k } \tilde { \mathbf { a } } _ { k }$ 7 in a continuous manner. We show in Figure 11.3 the accuracy gain of integration with both piecewise-linear and GP-based continuous representations compared to the Euler method shown in Figure 11.2. With the piecewise-linear approximation, the first integral (from $\hat { \mathbf { a } } _ { k }$ to $\Delta { \pmb v } _ { i k } )$ corresponds to the classic trapezoidal rule for numerical integration. This model can be interpreted as a constant-jerk motion model and already provides a significant accuracy gain compared to the Euler method. Going further in the quest for model-less integration, using GP regression with $\hat { \mathbf { a } } \sim \mathcal { G P } \left( 0 , k _ { \mathbf { a } } ( t , t ^ { \prime } ) \mathbf { I } \right)$ and $k _ { \mathbf { a } } ( t , t ^ { \prime } )$ is the square exponential covariance kernel function, the direct analytical inference of the integral (and double integral) of aˆ is enabled by the application of linear operators on GPs [969]. Accordingly, the method does not rely on any explicit motion model as the square exponential kernel is infinitely diferentiable. The bottom row of Figure 11.3 shows how the nonparametric GP model improves the integration accuracy compared to the piecewiselinear method. Note that the kernel’s hyperparameters control the smoothness of the signal and can be learned from the data or be set with an educated guess.

![](images/626e47f69d40952c268571a22990f598d593487a92f9598918b020fb88de18d7.jpg)  
Figure 11.3 Top row: Continuous integration with piecewise-linear approximation (corresponding to constant-jerk motion assumption). Bottom row: Model-free integration with Gaussian Process regression.

## 11.2.3.3 Continuous Rotation Preintegration

Looking at the accuracy gain brought by continuous representations for the translation and velocity preintegration, we naturally want to extend the concept to the rotation part. However, integrating over the space of rotations is challenging as the rotations R belong to the SO(3) Lie group, which is not an Euclidean space. Properties like the commutativity of the group operation do not hold for rotations.

Indeed, the product integral

$$
\boldsymbol {R} _ {b} ^ {w} (t + \Delta t) = \boldsymbol {R} _ {b} ^ {w} (t) \prod_ {t} ^ {t + \Delta t} \mathrm{Exp} \left(\boldsymbol {\omega} _ {b} ^ {b} (\tau)\right) ^ {d \tau}\tag{11.35}
$$

that solves the kinematic model (11.8) does not have a known generic solution [109] and novel approaches are required to perform continuous model-less integration over the space of rotations.

In response to this challenge, the authors of [630] propose to leverage the rotation vector representation $\mathbf { r } ( t )$ in the Lie algebra (with $R ( t ) = \exp ( { \mathbf { r } ( t ) } ) )$ as a linear vector space to perform continuous integration using linear tools. In that space, the system’s dynamics is

$$
\dot {\mathbf {r}} = \left(\mathrm{J} _ {r} (\mathbf {r})\right) ^ {- 1} \boldsymbol {\omega} _ {b} ^ {b},\tag{11.36}
$$

where $\bar { \mathbf { J } } _ { r } ( \mathbf { r } )$ is the right-hand Jacobian of SO(3) evaluated at r. Unfortunately, neither r nor r˙ are directly observed by the IMU. The key idea of [630] is to model r˙ with a GP and a set of virtual observations $\dot { \mathbf { r } } _ { t _ { \bullet } }$ to represent the continuous rotation vector function r via the use of linear operators on GPs. Intuitively, the virtual observations can be interpreted as control points of the continuous rotational dynamics. These are estimated through a non-linear least-square optimisation problem with residuals based on (11.36) and the gyroscope measurements as observations of $\omega _ { b } ^ { b }$ . This results in a model-less approach to continuous rotation preintegration and yields accuracy improvements of at least one order of magnitude over the standard discrete preintegration.

This continuous approach shares a lot of similarities with the STEAM continuoustime state estimation detailed in [47] and mentioned in Chapter 2 as both operate in the Lie algebra to perform GP-based interpolation. A major diference is the use of the square exponential kernel that results in a dense linear system, compared to the sparse Markovian approach used in STEAM. However, for the sake of IMU preintegration the length of an integration window is generally short enough that solving a dense system is not an issue. The concept of optimized inducing values is extended in [376] to also estimate the rotation-corrected acceleration along with the rotation vector. This allows to correlate the rotation and translation parts of the preintegrated measurement covariance matrix.

## 11.3 Observability of Aided Inertial Navigation

As we mentioned earlier in this chapter, due to measurement noise, biases, and inaccuracies of numerical integration, pure inertial odometry may drift quickly, in particular when using low-fidelity inertial sensors. A common approach to reduce the drift is to pair the IMU with exteroceptive sensors, $e . g .$ , cameras or LiDARs, leading to aided INS (AINS). In many cases, the introduction of exteroceptive sensors further increases the size of the state we have to estimate, $e . g . ,$ by adding extra variables corresponding to external landmarks, hence a natural question to ask is whether the sensor data is suficient to unambiguously estimate the SLAM state of the system. This is the goal of the observability analysis, which ascertains whether the information provided by the available measurements is suficient for estimating the state/parameters without ambiguity [117, 453].

The observability analysis is typically performed by deriving linearized measurement models and computing the observability matrix, which is closely related to the Fisher information (and covariance) matrix of the state estimate [487, 485] $( c f .$ Chapter 6). When a system is observable, the observability matrix is full-rank; if not, as this matrix describes the information available in the measurements, studying its nullspace enables us to gain insights about the directions in the state space along which the estimator lacks suficient information. The results of the observability analysis can be used to improve estimation consistency [1234, 456, 653], determine the minimal measurements needed to initialize an estimator [456, 737], and also identify degenerate motions that cause additional unobservable directions and should be avoided or alerted if possible in practice [1235]. For these reasons, signifi cant research eforts have been devoted to the observability analysis of AINS [1234], and in particular visual-inertial systems $( e . g .$ , [457, 654, 1235]).

In this section we discuss observability properties when the sensors used to aid the IMU produces geometric features, including points, lines, and planes. This general treatment allows discussing observability for a broad range of sensors, including cameras and LiDARs, and understanding degenerate configurations. In particular, Section 11.3.1 introduces linearized models assuming exteroceptive measurements of geometric landmarks, Section 11.3.2 uses these models to perform the observability analysis, and Section 11.3.3 discusses degenerate configurations.

## 11.3.1 Linearized Measurement Models

We describe the measurement models assuming that the sensor $( e . g .$ , camera, LiDAR) used to aid the IMU produces geometric features; in other words, we focus on SLAM and odometry front-ends which produce landmark-based representations. While most AINS use point features, in particular when relying on cameras (e.g., [456, 653, 645, 896, 375, 335]), line and plane features can be utilized when available (e.g., [599, 455, 414, 1236]). In such a case, we may need to augment the to-be-estimated state vector with all these diferent geometric features. Specifically, the AINS state that we are trying to estimate (at each time step) includes both the state of the robot $\scriptstyle { \mathbf { { \mathit { x } } } } _ { b }$ and the state of external features $\pmb { x } _ { \mathrm { f } } ^ { w }$ (expressed in the world frame):

$$
\boldsymbol {x} = \left\{\boldsymbol {R} _ {b} ^ {w}, \mathbf {b} ^ {g}, \boldsymbol {v} ^ {w}, \mathbf {b} ^ {a}, \boldsymbol {p} ^ {w}, \boldsymbol {x} _ {\mathrm{f}} ^ {w} \right\}\tag{11.37}
$$

where $R _ { b } ^ { w }$ is the rotation of the body frame $\mathcal { F } ^ { b }$ with respect to the world frame ${ \mathcal { F } } ^ { w }$ , and $\mathbf { \Delta } _ { p } w \mathbf { \Delta } , v ^ { w }$ are the robot position and velocity expressed in the world frame, respectively, while $ { \mathbf { b } } ^ { g } ,  { \mathbf { b } } ^ { a }$ are the gyroscope and accelerometer biases in the body frame. The features $\pmb { x } _ { \mathrm { f } } ^ { w }$ can be either points, lines, or planes (or a combination thereof) and are expressed in the world frame.

For the ensuing observability analysis, we need both the system dynamic model (which is related to the accelerations and angular rate measurements of the IMU) and the exteroceptive measurement model. Below, we start by reviewing the INS kinematic model —building on the IMU equations introduced in the previous section— and then consider exteroceptive measurement equations.

## 11.3.1.1 Linearized IMU Kinematic Model

The IMU-based kinematic model is given by (cf. (11.8) and (11.32)):

$$
\dot {\pmb {R}} _ {b} ^ {w} = \pmb {R} _ {b} ^ {w} (\pmb {\omega} _ {b} ^ {b}) ^ {\wedge}, \quad \dot {\pmb {v}} ^ {w} = \mathbf {a} ^ {w}, \quad \dot {\pmb {p}} ^ {w} = \pmb {v} ^ {w},\tag{11.38}
$$

$$
\dot {\mathbf {b}} ^ {g} (t) = \boldsymbol {\eta} ^ {b g}, \quad \dot {\mathbf {b}} ^ {a} (t) = \boldsymbol {\eta} ^ {b a}\tag{11.39}
$$

where $\eta ^ { b g }$ and $\eta ^ { b a }$ are the zero-mean Gaussian noises driving the gyroscope and accelerometer biases (which are modeled as random walks). In order to perform the observability analysis, we linearize the above nonlinear system and obtain the following continuous-time linearized error-state dynamical system:

$$
\dot {\tilde {\boldsymbol {x}}} (t) \simeq \left[ \begin{array}{c c} \mathbf {F} _ {c} (t) & \mathbf {0} _ {1 5 \times n _ {\mathrm{f}}} \\ \mathbf {0} _ {n _ {\mathrm{f}} \times 1 5} & \mathbf {0} _ {n _ {\mathrm{f}}} \end{array} \right] \tilde {\boldsymbol {x}} (t) + \left[ \begin{array}{c} \mathbf {G} _ {c} (t) \\ \mathbf {0} _ {n _ {\mathrm{f}} \times 1 2} \end{array} \right] \boldsymbol {\eta} (t) =: \mathbf {F} (t) \tilde {\boldsymbol {x}} (t) + \mathbf {G} (t) \boldsymbol {\eta} (t)\tag{11.40}
$$

where the error-state vector $\tilde { \mathbf { \ b { x } } } = \{ \tilde { \theta } , \tilde { \mathbf { b } } ^ { g } , \tilde { \mathbf { v } } ^ { w } , \tilde { \mathbf { b } } ^ { a } , \tilde { \mathbf { p } } ^ { w } , \tilde { \mathbf { x } } _ { \mathrm { { t } } } ^ { w } \}$ (expressed as a column vector) represents the deviation from the linearization point $( e . g . , \tilde { \mathbf { b } } ^ { g }$ is the change of the bias with respect to the linearization point), and for the rotation component we use the tangent-space representation $\tilde { \pmb { \theta } }$ at the linearization point.<sup>8</sup> In (11.40), $n _ { \mathrm { f } }$ is the dimension of $\tilde { { \mathbfcal x } } _ { \mathrm { f } } ^ { w } , \mathbfcal F _ { c } ( t )$ and ${ \bf G } _ { c } ( t )$ are the continuous-time linearized transition matrix and the noise Jacobian matrix for the IMU state, respectively, and $\pmb { \eta } ( t )$ is the stacked noise, including both $\eta ^ { b g }$ and $\eta ^ { b a }$ as well as the IMU noise which arises when substituting the actual acceleration and rotation rates in (11.38) with the accelerometer and gyroscope measurements $( c f .$ with derivation in Section 11.2.1).

As in practice AINS estimators are typically implemented in discrete time, the discrete-time dynamic model is needed and can be derived by computing its state transition matrix $\Phi _ { ( k + 1 , k ) }$ from time $t _ { k }$ to $t _ { k + 1 }$ , based on $\dot { \Phi } _ { ( k + 1 , k ) } = \mathbf { F } ( t _ { k } ) \Phi _ { ( k + 1 , k ) }$

with identity as the initial condition:

$$
\boldsymbol {\Phi} _ {(k + 1, k)} = \left[ \begin{array}{c c c c c c} \boldsymbol {\Phi} _ {1 1} & \boldsymbol {\Phi} _ {1 2} & \mathbf {0} _ {3} & \mathbf {0} _ {3} & \mathbf {0} _ {3} & \mathbf {0} _ {n _ {\mathrm{f}} \times 3} \\ \mathbf {0} _ {3} & \mathbf {I} _ {3} & \mathbf {0} _ {3} & \mathbf {0} _ {3} & \mathbf {0} _ {3} & \mathbf {0} _ {n _ {\mathrm{f}} \times 3} \\ \boldsymbol {\Phi} _ {3 1} & \boldsymbol {\Phi} _ {3 2} & \mathbf {I} _ {3} & \boldsymbol {\Phi} _ {3 4} & \mathbf {0} _ {3} & \mathbf {0} _ {n _ {\mathrm{f}} \times 3} \\ \mathbf {0} _ {3} & \mathbf {0} _ {3} & \mathbf {0} _ {3} & \mathbf {I} _ {3} & \mathbf {0} _ {3} & \mathbf {0} _ {n _ {\mathrm{f}} \times 3} \\ \boldsymbol {\Phi} _ {5 1} & \boldsymbol {\Phi} _ {5 2} & \boldsymbol {\Phi} _ {5 3} & \boldsymbol {\Phi} _ {5 4} & \mathbf {I} _ {3} & \mathbf {0} _ {n _ {\mathrm{f}} \times 3} \\ \mathbf {0} _ {3 \times n _ {\mathrm{f}}} & \mathbf {0} _ {3 \times n _ {\mathrm{f}}} & \mathbf {0} _ {3 \times n _ {\mathrm{f}}} & \mathbf {0} _ {3 \times n _ {\mathrm{f}}} & \mathbf {0} _ {3 \times n _ {\mathrm{f}}} & \mathbf {I} _ {n _ {\mathrm{f}}} \end{array} \right]\tag{11.41}
$$

where the $( i , j )$ block $\Phi _ { i j }$ can be found analytically or numerically [456].

## 11.3.1.2 Exteroceptive Measurement Models

Now we present the measurement models of diferent geometric features and their linearized models, which are essential for the linearized AINS observability analysis.

Point Features. Consider point feature measurements provided by exteroceptive sensors (such as monocular/stereo camera, acoustic sonar, and LiDAR). These can generally be modeled as range and/or bearing observations, which are functions of the relative position of the feature in the sensor frame ${ \mathcal { F } } ^ { c }$ :

$$
\boldsymbol {z} _ {p} = \underbrace {\left[ \begin{array}{c c} \lambda_ {r} & \mathbf {0} _ {1 \times 2} \\ \mathbf {0} _ {2 \times 1} & \lambda_ {b} \mathbf {I} _ {2} \end{array} \right]} _ {\boldsymbol {\Lambda}} \left[ \begin{array}{c} z _ {r} \\ \boldsymbol {z} _ {b} \end{array} \right] = \boldsymbol {\Lambda} \left[ \begin{array}{c} \| \boldsymbol {p} _ {\mathrm{f}} ^ {c} \| + \eta^ {r} \\ h _ {b} (\boldsymbol {p} _ {\mathrm{f}} ^ {c}) + \boldsymbol {\eta} ^ {b} \end{array} \right]\tag{11.42}
$$

where $\pmb { p } _ { \mathrm { f } } ^ { c } = \pmb { R } _ { w } ^ { c } \left( \pmb { p } _ { \mathrm { f } } ^ { w } - \pmb { p } _ { c } ^ { w } \right)$ is the position of the feature in the sensor frame, and $z _ { r }$ and $z _ { b }$ denote range and bearing measurements, respectively. In particular, $h _ { b } ( \cdot )$ is a generic bearing measurement function whose actual form depends on the particular sensor used. Λ is a measurement selection matrix, with binary entries $\lambda _ { r }$ and $\lambda _ { b } ;$ for example, if $\lambda _ { b } = 1$ and $\lambda _ { r } = 1$ , then $z _ { p }$ contains both range and bearing measurements. $\eta ^ { r }$ and $\eta ^ { b }$ are the measurement noises and are assumed to be additive for simplicity. Linearizing (11.42) with the chain rule of diferentiation at the current state estimate yields the following measurement error equation:

$$
\tilde {\boldsymbol {z}} _ {p} = \boldsymbol {z} _ {p} - \hat {\boldsymbol {z}} _ {p} \simeq \boldsymbol {\Lambda} \left[ \begin{array}{l} \frac {\partial z _ {r}}{\partial \boldsymbol {p} _ {\mathrm{f}} ^ {c}} \frac {\partial \boldsymbol {p} _ {\mathrm{f}} ^ {c}}{\partial \boldsymbol {x}} \\ \frac {\partial \boldsymbol {z} _ {b}}{\partial \boldsymbol {p} _ {\mathrm{f}} ^ {c}} \frac {\partial \boldsymbol {p} _ {\mathrm{f}} ^ {c}}{\partial \boldsymbol {x}} \\ \end{array} \right] _ {\hat {\boldsymbol {x}}} ^ {\boldsymbol {\tilde {x}}} + \eta^ {r} =: \boldsymbol {\Lambda} \left[ \begin{array}{l} \mathbf {H} _ {r} \\ \mathbf {H} _ {b} \end{array} \right] \mathbf {H} _ {\mathrm{f}} \tilde {\boldsymbol {x}} + \boldsymbol {\Lambda} \left[ \begin{array}{l} \eta^ {r} \\ \eta^ {b} \end{array} \right] =: \mathbf {H} _ {x} \tilde {\boldsymbol {x}} + \eta^ {p}\tag{11.43}
$$

where $\hat { z } _ { p }$ is the measurement at the linearization point. Depending on the selection matrix Λ, the Jacobian $\mathbf { H } _ { x }$ may include the range-only measurement Jacobian $\mathbf { H } _ { \boldsymbol { r } }$ $( \lambda _ { r } = 1 , \lambda _ { b } = 0 )$ , the bearing-only Jacobian ${ \bf H } _ { b } \ ( \lambda _ { r } = 0 , \lambda _ { b } = 1 )$ , or both.

Line Features. Given two 3D points ${ \bf p } _ { 1 } ^ { w }$ and $\mathbf { p } _ { 2 } ^ { w }$ , we can represent the line passing through the two points using its Pl¨ucker coordinates:

$$
\mathbf {l} ^ {w} = \left[ \begin{array}{c} \mathbf {n} _ {\ell} ^ {w} \\ \mathbf {v} _ {\ell} ^ {w} \end{array} \right] = \left[ \begin{array}{c} \mathbf {p} _ {1} ^ {w} \times \mathbf {p} _ {2} ^ {w} \\ \mathbf {p} _ {2} ^ {w} - \mathbf {p} _ {1} ^ {w} \end{array} \right]\tag{11.44}
$$

where $\mathbf { n } _ { \ell } ^ { w }$ is the line moment that encodes the normal direction of the plane defined by the two points and the origin, and ${ \bf v } _ { \ell } ^ { w }$ is the line direction vector which can be normalized to a unit vector if needed. Note that the distance from the origin to the line can be computed as $\begin{array} { r } { d _ { \ell } ^ { w } = \frac { \| \mathbf { n } _ { \ell } ^ { w } \| } { \left\| \mathbf { v } _ { \ell } ^ { w } \right\| } } \end{array}$ , and the above Pl¨ucker coordinate —expressed in the world frame— can be transformed to the camera frame as [1021]:

$$
\left[ \begin{array}{c} \mathbf {n} _ {\ell} ^ {c} \\ \mathbf {v} _ {\ell} ^ {c} \end{array} \right] = \left[ \begin{array}{c c} \boldsymbol {R} _ {c} ^ {w ^ {\top}} & - \boldsymbol {R} _ {c} ^ {w ^ {\top}} (\boldsymbol {p} _ {c} ^ {w}) ^ {\wedge} \\ \boldsymbol {0} & \boldsymbol {R} _ {c} ^ {w ^ {\top}} \end{array} \right] \left[ \begin{array}{c} \mathbf {n} _ {\ell} ^ {w} \\ \mathbf {v} _ {\ell} ^ {w} \end{array} \right]\tag{11.45}
$$

We now consider a case where the 3D line is observed in 2D images. Specifically, given two endpoints of a line segment in the image: $\mathbf { q } _ { 1 } : = [ u _ { 1 } , v _ { 1 } , 1 ] ^ { \mathsf { T } }$ and ${ \bf q } _ { 2 } : =$ $[ u _ { 2 } , v _ { 2 } , 1 ] ^ { \mathsf { T } }$ , we derive the 2D visual line measurement model as the distances of these two endpoints to the back-projected 3D Pl¨ucker line onto the image plane [1315]. To this end, we transform the 3D line from the world frame to the current camera frame via (11.45) and then project it onto the image with the known intrinsic parameters of the camera [1021]:

$$
\boldsymbol {\ell} = \underbrace {\left[ \begin{array}{c c c} f _ {2} & 0 & 0 \\ 0 & f _ {1} & 0 \\ - f _ {2} c _ {1} & - f _ {1} c _ {2} & f _ {1} f _ {2} \end{array} \right]} _ {\mathbf {K}} \left[ \begin{array}{c c} \mathbf {I} _ {3} & \mathbf {0} _ {3} \end{array} \right] \left[ \begin{array}{c} \mathbf {n} _ {\ell} ^ {c} \\ \mathbf {v} _ {\ell} ^ {c} \end{array} \right] =: \left[ \begin{array}{c} \ell_ {1} \\ \ell_ {2} \\ \ell_ {3} \end{array} \right]\tag{11.46}
$$

where K is the canonical projection Pl¨ucker matrix and $f _ { 1 } , f _ { 2 } , c _ { 1 }$ and $c _ { 2 }$ are the standard camera intrinsic parameters. Note that only the moment vector ${ \bf n } _ { \ell } ^ { c }$ in the Pl¨ucker coordinates is involved in the above projection, which implies that the line range and orientation contained in $\mathbf { v } _ { \ell } ^ { c }$ are not measurable. Therefore, the distances of the two endpoints of the line segment to the projected line ℓ in the image can be finally computed and used as the line feature measurements:

$$
\boldsymbol {z} _ {\ell} = \left[ \begin{array}{c} \frac {\mathbf {q} _ {1} ^ {\top} \boldsymbol {\ell}}{\sqrt {\ell_ {1} ^ {2} + \ell_ {2} ^ {2}}} \\ \frac {\mathbf {q} _ {2} ^ {\top} \boldsymbol {\ell}}{\sqrt {\ell_ {1} ^ {2} + \ell_ {2} ^ {2}}} \end{array} \right] + \boldsymbol {\eta} ^ {\ell}\tag{11.47}
$$

where $\eta ^ { \ell }$ is the measurement noise. Similarly, with the chain rule of diferentiation, we can linearize (11.47) with respect to the state and obtain the measurement Jacobian: $\begin{array} { r } { \mathbf { H } _ { x } = \frac { \partial z _ { \ell } } { \partial \ell } \frac { \partial \ell } { \partial \pmb { x } } } \end{array}$

Plane Features. A 3D plane can be parameterized by its distance to the origin and normal direction in the world frame: ${ \boldsymbol { \pi } } ^ { w } = \left[ { \bf { n } } _ { \pi } ^ { w } \right]$ , which can be transformed to the local sensor frame where the plane feature is typically detected:

$$
\left[ \begin{array}{c} \boldsymbol {n} _ {\pi} ^ {c} \\ d _ {\pi} ^ {c} \end{array} \right] = \left[ \begin{array}{c c} \boldsymbol {R} _ {w} ^ {c} & \mathbf {0} _ {3 \times 1} \\ - (\boldsymbol {p} _ {c} ^ {w}) ^ {\top} & 1 \end{array} \right] \left[ \begin{array}{c} \boldsymbol {n} _ {\pi} ^ {w} \\ d _ {\pi} ^ {w} \end{array} \right]\tag{11.48}
$$

Without loss of generality, we consider a plane feature $( n _ { \pi } ^ { c } , d _ { \pi } ^ { c } )$ is extracted from point clouds (e.g., LiDAR or depth sensors), and employ the closes point $p _ { \pi } ^ { c } \ : =$ $d _ { \pi } ^ { c } { \pmb { n } } _ { \pi } ^ { c }$ from the plane to the origin as the plane representation in the AINS state vector [374].

$$
\pmb {z} _ {\pi} = d _ {\pi} ^ {c} \pmb {n} _ {\pi} ^ {c} + \pmb {\eta} ^ {\pi} = \pmb {p} _ {\pi} ^ {c} + \pmb {\eta} ^ {\pi}\tag{11.49}
$$

where $\eta ^ { \pi }$ is the plane measurement noise. Linearization of (11.49) yields the plane measurement Jacobian $\begin{array} { r } { \mathbf { H } _ { x } = \frac { \partial \pmb { z } _ { \pi } } { \partial \pmb { p } _ { \pi } ^ { c } } \frac { \partial \pmb { p } _ { \pi } ^ { c } } { \partial \pmb { x } } } \end{array}$

## 11.3.2 Observability Analysis

Based on the linearized system and measurement models presented in the previous sections, we can now perform the observability analysis. The analysis relies on the following observability matrix M(xˆ) to gain insights about the system (cf. [486]):

$$
\mathbf {M} (\hat {\boldsymbol {x}}) = \left[ \begin{array}{c} \mathbf {H} _ {x _ {1}} \boldsymbol {\Phi} _ {(1, 1)} \\ \mathbf {H} _ {x _ {2}} \boldsymbol {\Phi} _ {(2, 1)} \\ \vdots \\ \mathbf {H} _ {x _ {k}} \boldsymbol {\Phi} _ {(k, 1)} \end{array} \right]\tag{11.50}
$$

where $\mathbf { H } _ { x _ { k } }$ stacks the Jacobians for all the measurements (of points, lines, or planes) collected at discrete time $k ,$ and the notation M(xˆ) stresses the fact that the observability matrix depends on the linearization point xˆ. The nullspace of this matrix, $i . e .$ , the span of the null vectors span $( [ \cdots ~ { \pmb u } _ { i } ~ \cdots ] ) = \mathcal { U }$ such that $\mathbf { M } ( \pmb { x } ) \pmb { u } _ { i } = \mathbf { 0 }$ 7 describes the unobservable subspace of AINS. If the nullspace is empty, the system is fully observable. It has been shown in [1234] that AINS in general has 4 unobservable directions (i.e., it has four independent vectors in the null space ), describing the fact that the global 3D position and global yaw are unobservable from IMU measurements and local observations of previously unknown landmarks.

To understand the structure of the 4-dimensional null space, we consider the case where all three types of geometric features (i.e., a single point, line, and plane) are in the state vector: $\pmb { x } _ { \mathrm { f } } ^ { w } = \{ \pmb { p } _ { \mathrm { f } } ^ { w } , \pmb { \mathrm { I } } ^ { w } , \pmb { \pi } ^ { w } \}$ , and the exteroceptive measurements include: $\boldsymbol { z } ~ = ~ \{ z _ { p } , z _ { \ell } , z _ { \pi } \}$ (cf. (11.42), (11.47), and (11.49)). By computing the related system and measurement Jacobians $( i . e . , \mathbf { H } _ { x _ { i } }$ and $\Phi _ { ( i , 1 ) } )$ and substituting them into (11.50), we can build the corresponding linearized AINS observability matrix M. By mathematically computing the nullspace of this matrix null(M), we should be able to find the following four null-vectors (cf. [1234]):

$$
\mathrm{null} (\mathbf {M}) = \mathrm{span} [ \boldsymbol {u} _ {1} \boldsymbol {u} _ {2: 4} ] = \mathrm{span} \left[ \begin{array}{c c} \boldsymbol {u} _ {g} & \mathbf {0} _ {1 2 \times 3} \\ - \boldsymbol {p} _ {1} ^ {w} \times \mathbf {g} ^ {w} & \mathbf {I} _ {3} \\ - \boldsymbol {p} _ {\mathrm{f}} ^ {w} \times \mathbf {g} ^ {w} & \mathbf {I} _ {3} \\ - \mathbf {g} ^ {w} & \frac {\mathbf {v} _ {\ell} ^ {w}}{d _ {\ell} ^ {w} | | \mathbf {v} _ {\ell} ^ {w} | |} (\boldsymbol {R} _ {\ell} ^ {w} \mathbf {e} _ {1}) ^ {\top} \\ 0 & - (\boldsymbol {R} _ {\ell} ^ {w} \mathbf {e} _ {3}) ^ {\top} \\ - d _ {\pi} ^ {w} \mathbf {n} _ {\pi} ^ {w} \times \mathbf {g} ^ {w} & \mathbf {n} _ {\pi} ^ {w} (\boldsymbol {R} _ {\pi} ^ {w} \mathbf {e} _ {3}) ^ {\top} \end{array} \right]\tag{11.51}
$$

where $\begin{array} { r } { { \pmb u } _ { g } = \left[ ( { \pmb R } _ { w } ^ { c _ { 1 } } { \pmb g } ^ { w } ) ^ { \top } \quad { \pmb 0 } _ { 1 \times 3 } \quad - ( { \pmb v } _ { 1 } ^ { w } \times { \pmb g } ^ { w } ) ^ { \top } \quad { \pmb 0 } _ { 1 \times 3 } \quad \right] ^ { \top } , p _ { 1 } ^ { w } } \end{array}$ refers to the sensor position at the time $k = 1 , R _ { w } ^ { c _ { 1 } }$ is the rotation matrix from the sensor frame $C _ { 1 }$ at time $k = 1$ to the world frame $W ,$ , while $R _ { \pi } ^ { w }$ is a rotation matrix built using the plane normal vector ${ \bf n } _ { \pi } ^ { w }$ using Gram–Schmidt orthonormalization (cf. (11.6)), and $\begin{array} { r l } { \pmb { R _ { \ell } ^ { w } } = \left\lceil \frac { { \bf n } _ { \ell } ^ { w } } { | | { \bf n } _ { \ell } ^ { w } | | } \right. } & { { } \frac { { \bf v } _ { \ell } ^ { w } } { | | { \bf v } _ { \ell } ^ { w } | | } \quad \left. \frac { { \bf n } _ { \ell } ^ { w } } { | | { \bf n } _ { \ell } ^ { w } | | } \times \frac { { \bf v } _ { \ell } ^ { w } } { | | { \bf v } _ { \ell } ^ { w } | | } \right] } \end{array}$ is the rotation matrix constructed with the line normal and line direction. It is possible to see that the first null vector $\mathbf { \delta } \mathbf { u } _ { 1 }$ is related to the rotation around the gravity (and hence the yaw) and $\mathbf { \delta } \mathbf { \mathscr { u } } _ { 2 : 4 }$ to the motion of the robot. Readers are referred to [1234, 1233] for more detailed analysis.

In summary, the fact that the observability matrix admits a 4-dimensional null space (cf. (11.51)) correctly describes the fact that the global position and yaw of the system are not observable. Intuitively, none of the measurements (IMU data, measurements of unknown point, line, or plane landmarks) convey information about the global frame, with the exception of roll and pitch, which are observable from the accelerometer measurements of the gravity direction. This unobservability is common in $\mathrm { S L A M ^ { 9 } }$ and it not pathological: it only means that we can arbitrarily set the yaw and 3D origin of our world frame since we only have relative measurements for those variables. This unobservability would disappear when adding a sensor providing absolute measurements, e.g., a GPS. More concerning is that fact that for certain motions (and linearization points), the null space of the observability matrix can grow larger, creating additional unobservable dimensions. We explore this phenomenon below.

## 11.3.3 Degenerate Motions

Certain types of motions might induce additional unobservable directions for AINS (i.e., in addition to the 4 expected ones that we discussed above). This is of practical importance since these degenerate motions might lead to large errors in some directions of the state space and lead to navigation failures. The degenerate motions of AINS are summarized in Table 11.1 (see [1234] for a full derivation). Specifically, pure translation is degenerate for all feature types, causing the full global rotation to become unobservable. Intuitively, if the system is not rotating, we might confuse gravity measurements with accelerometer biases, hence making the roll and pitch no longer observable. The other three degenerate motions, namely constant acceleration (including the case of constant velocity, where the acceleration is set to zero), pure rotation, and motion in the direction of the feature (for the case where we have a single point feature), cause the scale to be unobservable for the case of monocular camera $( i . e . ,$ bearing-only measurements). However, constant acceleration causes the whole system $( i . e .$ , position, velocity, acceleration bias, and features) scale to be unobservable, while pure rotation and moving toward a feature only make the feature scale unobservable. Note that these three degenerate motions hold only if the distance from the sensor to the feature is significantly larger than the extrinsic translation between the sensor and robot body (if they do not coincide), i.e., $| | { \pmb p } _ { \mathrm { f } } ^ { c } | | > > | | { \pmb p } _ { b } ^ { c } | |$ , which typically is the case in practice.

Table 11.1 Degenerate motions of AINS.

<table><tr><td>Motion</td><td>Sensor</td><td>Unobservable</td></tr><tr><td>1. Pure translation</td><td>General</td><td>Global orientation</td></tr><tr><td>2. Constant acceleration</td><td>Mono cam</td><td>System scale</td></tr><tr><td>3. Pure rotation</td><td>Mono cam</td><td>Feature scale</td></tr><tr><td>4. Moving toward point feature</td><td>Mono cam</td><td>Feature scale</td></tr></table>

## 11.4 Visual-Inertial Odometry and Practical Considerations

As mentioned above, inertial measurements are typically fused with data from other sensors to mitigate the odometry drift. In this section, we particularly focus on the case where visual measurements from a camera are fused with IMU measurements using factor graphs.<sup>10</sup> Camera and IMU are a popular combination, since the are both inexpensive, lightweight, and low-power sensors. Moreover, they are complementary sensors, where the IMU is able to capture quick acceleration and rotations, while cameras are able to provide rich observations of the surrounding environment. On one side, the use of cameras largely reduces the drift as compared to pure inertial odometry; on the other side, an IMU might allow observing quantities that would not be possible to estimate otherwise. In particular, when using a monocular camera for SLAM, one cannot estimate the scale of the scene without relying on prior information (in other words, the scale is unobservable), while adding an IMU allows retrieving the scale as well, as long as the motion of the robot is non-degenerate (Section 11.3.3). As we mentioned earlier in this chapter, systems comprising one or more cameras as well as an IMU are typically referred to as visual-inertial odometry (VIO) systems, and become visual-inertial SLAM systems when loop closures are incorporated.

## 11.4.1 Visual-Inertial Odometry

VIO systems are commonly used as a source of odometry and are often used to close control loops over trajectory tracking and control. In other applications, such as virtual reality, VIO systems are used to compensate for the motion of the user in the virtual environment. In both cases, VIO is required to produce estimates with very low-latency, typically in the order of 10-50ms. For instance, the refresh rate of the Meta Quest 3 is between 72Hz and 120Hz [761], and the VIO latency directly impacts the quality of the VR experience and is key to mitigating motion sickness. Similarly, for trajectory tracking it is important to keep the latency low since large delays might induce instability and divergence of the tracking controller.

![](images/627be82498ae6fc3e36cf59a3d3824fd1a7c0fff6af8037eab8501c5ab27e5bd.jpg)  
Figure 11.4 Example of factor graph used for visual-inertial odometry with preintegrated IMU factors [335]. The factor graph shows preintegrated IMU factors in violet (constraining consecutive poses, velocity, and bias), bias factors in blue (constraining the evolution of the IMU biases over time), vision factors in orange (relating camera poses and positions of external landmarks), and priors in black.

Based on these consideration, factor-graph-based VIO systems typically implement a fixed-lag smoother (also called sliding-window optimization), where one only attempts to estimate states in a receding horizon (e.g., the last 5-10 seconds). An example of the resulting factor graph is shown in Fig. 11.4, which shows preintegrated IMU factors in violet, bias factors in blue, vision factors in orange, and priors in black. The horizon is chosen in a way to trade-of computation with accuracy, since the longer the horizon, the larger the state space for estimation. Then factors and variables falling out of the receding horizon are gradually marginalized as time progresses. In many cases, optimized implementations also eliminate visual landmarks from the optimization using the Schur complement to further reduce the size of the state space, see, e.g., [335]. An alternative to using a fixed-lag smoother is to use an incremental solver like iSAM2 (Section 1.7), which reuses computation from previous optimizations when computing an estimate at the current time. While this approach has been shown to lead to very accurate results in practice [335], it has the drawback of not providing guarantees on the latency of the system and might lead to spikes in the runtime, which is problematic for certain applications.

VIO Systems and Performance. The last decade has seen a proliferation of visual-inertial odometry/SLAM systems, many of which have open-source implementations. Popular approaches include a visual-inertial version of ORB-SLAM [786], Direct Sparse Visual-Inertial Odometry [1116], VINS-Mono [896], OpenVINS [375], Kimera [3, 942], BASALT [1118], and DM-VIO [1043]. A good VIO system has a drift below 1% of the distance traveled (e.g., it accumulates an error smaller than 1m after covering a 100m trajectory), and in some cases the drift can be as low as 0.1%.

![](images/64e94aa2c5720f69c4c2a8b87d98f85d571e1ddf16e4f47ed45bf87dd3c232fc.jpg)  
Figure 11.5 Illustration of a recent sliding-window optimization-based VIO algorithm [180] running on the KAIST urban autonomous driving dataset, sequence 38. This sequence has a total duration of 36 minutes and is 11.42 km in length. The VIO estimates and ground truth are overlaid on the Google map. Bottom are two sample images. The final ATE of the VIO (i.e., without loop closure) is 2.05 degrees and 21.2 meters (0.18%).

Sliding-window optimization approaches such as VINS-Mono [896] have seen tremendous success in practice. As an example, here we show how a more recent sliding-window-based approach, called First-Estimate Jacobian (FEJ)-based Window Bundle Adjustment (WBA)-VINS [180, 181] performs on the KAIST Urban Dataset [515]. The KAIST urban dataset focuses on autonomous driving and localization in challenging complex urban environments. The dataset was collected in Korea with a vehicle equipped with stereo camera pair, 2D/3D LiDARs, Xsens IMU, Fiber Optic Gyro (FoG), wheel encoders, and RKT GPS. The camera operates at 10Hz, while the IMU sensing rate is 100Hz. A ground-truth trajectory is also provided which is obtained from the fusion of the FoG, RKT GPS, and wheel encoders. Fig. 11.5 illustrates the FEJ-WBA-VINS [180, 181] (VIO) estimated trajectory for sequence 38, overlaid on a Google map alongside the ground truth (GT) trajectory. The final absolute trajectory error (ATE) for the 11.42-kilometer path is approximately 2.05 degrees and 21.2 meters (0.18% of the trajectory traveled). Notably, these results are obtained from the pure online VIO without loop closure.

Applications of VIO to self-driving cars and other autonomous systems is discussed in [3, 4], which also discusses challenges related to feature tracking, keyframe section, and fusion of diferent sensor modalities (including monocular, stereo, and RGB-D camera images, as well as wheel odometry).

## 11.4.2 Extrinsic Calibration

In order to enable accurate aided inertial navigation, one needs to perform extrinsic calibration of the sensors. The extrinsic calibration corresponds to estimating the relative pose between the diferent sensors (e.g., the pose of the camera with respect to the IMU). The literature provides a variety of methods to address the calibration problem. These can be mainly divided into two categories: ofline and online calibration methods. Ofline approaches require a calibration procedure to be performed before the system is deployed. It often involves the use of a calibration target [350], a known motion pattern [686], or prior knowledge about the environment [631, 714]. These procedures can be more-or-less time consuming, and may require specific equipment and trained operators. Thus, while generally more accurate, ofline calibration methods can be cumbersome and undesirable in some scenarios where the final system is expected to be used at large scale by non-experts. Online methods, on the other hand, do not require a specific procedure [301, 633, 1211, 1237]. Instead, the extrinsic calibration parameters are estimated as part of the state estimation problem. This ofers the advantage of being able to adapt to changes in the system, such as sensor displacement, without the need for a new calibration procedure. However, online calibration methods can be less accurate than ofline methods, and may render the state estimation problem more complex or even ill-posed [1235].

## 11.4.3 Temporal Synchronization

Another crucial aspect of inertial-aided system is the temporal synchronization of the sensor data. An erroneous synchronization that is not accounted for can lead to significant errors in the trajectory estimates and/or introduce a bias in the benchmarking metrics. The synchronization can be done either in hardware or in software. The low-level hardware approach often relies on a dedicated piece of hardware that triggers the various sensors’ data acquisition based on a common clock signal via specific synchronization input pins. This is not always possible, especially when the sensors are connected to the computer via diferent communication protocols. Some sensors may have a built-in synchronization mechanism that can be used to synchronize the diferent sensors’ clock without the need for a dedicated hardware input. The use of PTP (Precision Time Protocol) is an example of such a softwarebased synchronization over Ethernet. Many LiDARs, radars and INS solutions can be synchronized with this protocol. Another solution is time-stamping the data at the sensor level and then aligning the timestamps in a post-processing step. This last approach is generally less accurate and robust than the aforementioned ones. If the system cannot be synchronized and post-processing is not an option (e.g., online applications), some state estimation algorithms integrate the time ofset as a state variable in the estimation problem [301, 376, 1237].

## 11.5 Further Readings & Recent Trends

While progress in inertial odometry is steadily transitioning into industry products, aided inertial navigation is still the subject of intense research.

Extended Pose Preintegration. Latest trends in inertial odometry for SLAM include the use of extended-pose manifolds and higher-order noise propagation [120] to improve the uncertainty modeling of IMU preintegration. Brossard et al. [120] extend the preintegration theory to account for Earth’s rotation with the Coriolis and centrifugal forces. Vial et al. [1126] provide an example of extended pose preintegration leveraging both a linear velocity sensor and a navigation-grade IMU. After an hour of marine navigation over a 1.8km trajectory, the authors report a translation error of around 5m.

Continuous-time State Representations. We have mostly been interested in the use of IMUs under the scope of preintegration as a mean to reduce the number of discrete state variables in our factor graphs. However, other approaches bases on continuous-time state representation can also account from many IMU measurements without increasing the dimensionality of the estimated state. We find such examples in [349] using B-splines basis functions and in [47] with GP priors. Both formulations allow to use IMU measurements at high rates in residuals based on interpolated dynamics between a fixed set of state variables. Recently, the work [130] compares the integration of IMU measurements directly as inputs in the continuous-time GP prior against using IMU measurements directly in residuals. They concluded that using inertial information as measurements of the state resulted in better odometry accuracy using a LiDAR-inertial sensor suite. Another interesting work on continuous-time representations is [659], where the authors compare the GP-based state representation from [47] with the continuous GP-based preintegration from [376] (presented earlier in this chapter). In a event-based VIO context, the authors show that the later provides a slight advantage over the former both in terms of accuracy and computational eficiency.

Proprioception-only Odometry. Recent works use proprioceptive sensors for aided inertial navigation. For odometry, works like [441] with legged robots and [813] with wheel-mounted IMUs demonstrate how some knowledge about the system’s kinematics can be used to provide competitive IMU-based odometry estimates with sub-percent positional error. In [441] the critical information is the knowledge of contact between the robot’s feet and the ground, while in [813] the one-planerotation motion is used to constrain the IMU biases and therefore limit the deadreckoning drift. The work [813] has been extended into a full SLAM system [1200] by detecting loop closures based on pattern recognition in the road bank angle over time, providing an interesting example of an IMU-based proprioceptive system that can perform loop closure detection and correction. It is important to note that while the use of inertial sensors generally ofers better performance and robustness, dropouts or saturation of the IMU sensor can have catastrophic efects on the overall system’s performance. In [266] the authors investigate the use of accelerometer data to estimate angular velocity when the gyroscope saturates, thus improving the robustness of downstream SLAM algorithms.

Inertial-only Odometry (IOO). Naive integration of IMU measurements — without aiding sources such as vision— typically leads to quick divergence of the odometric estimate. This is a cause of concern even in aided inertial odometry when the source of aiding becomes unavailable. For example, for hand tracking in mobile AR/VR applications, highly dynamic hands can easily move out of the tracking camera’s FOV, leaving only IMU data available to keep motion tracking alive; or textureless scenes may prevent feature detection and tracking, causing VIO to only rely on IMU data. For this reason, recent work investigates the use of learning and neural networks to reduce the drift in inertial-only odometry [1215, 179, 1055, 451, 452, 220, 898]. These include attempts to model IMU bias in a data-driven manner with neural networks [225] or directly predicting displacements from a sequence of noisy IMU measurements [682]. For instance, one may use a diferentiable integration module to integrate IMU readings with the predicted bias removed [1276, 898], or directly use ground truth bias for supervision [123], or use a conditional difusion model to approximate bias which is modeled as a probability distribution [1298]. These methods have demonstrated the possibility of largely reducing the drift in inertial-only odometry, but currently they have limited generalization (e.g., to diferent sensors or to motions not seen at training time).

Ultra-eficient and Robust VIO at the Edge. Despite recent advancements in SLAM, computational constraints arising in embedded robotic systems still pose critical challenges. Building robust VIO on these small form-factor platforms is hard due to strict size, weight, and power (SWAP) constraints, with the primary dificulty often arising from data management rather than computation. For example, in SLAM and hand-tracking modules of Meta XR wearable devices, the major energy consumption is data access in RAM [623]. To reduce the data transfer, an on-sensor computing architecture is presented [390], and a quantized visual-inertial odometry (QVIO) algorithms is developed in [865, 867]. For low-SWaP platforms, where only single-precision floating-point arithmetic is available on the computation unit or is required to speed up and achieve real-time performance, new square-root (information or covariance) filters [866, 1194] have been introduced to improve eficiency while maintaining numerical stability. An ASIC design and implementation for on-chip visual-inertial odometry system is presented in [1279, 1048].

Leg Odometry for SLAM Marco Camurri and Mat´ıas Mattamala

Legged robots are becoming widespread thanks to their ability to traverse highly unstructured terrains. Their main advantage is that legs provide an active suspension that decouples the motion of the robot’s body from the terrain profile [988]. This enables them to negotiate staircases, uneven terrain, and other ground obstacles that are challenging for wheeled platforms [218, 500]. Even though the SLAM algorithms reviewed in the other chapters are directly applicable to legged robots, the additional sensing introduced by the legs is a valuable new source of information that can be exploited for odometry. This is particularly important for legged locomotion control and planning, where high-frequency and low drift real-time pose and velocity estimation is required to prevent falls and failures, which is challenging.

In this chapter, we discuss the foundations of estimating the real-time pose and velocity of a legged robot equipped with an onboard IMU and joint sensing (position and torque). We will particularly focus on leg odometry, which aims to determine the relative motion of the robot’s body from leg sensing. We start by providing some historical background and preliminaries (Section 12.1). Then we introduce the key theoretical tools used to estimate leg odometry (Section 12.2 and Section 12.3). In particular, we explain how the relative motion can be obtained from the joint sensing of the legs, assuming that the stance legs are known (Section 12.2). This practically allows us to consider leg odometry as a measurement in our SLAM problem, in a similar way to wheel odometry or inertial preintegration. Then, we describe how contact is estimated, and the main techniques adopted in practice (Section 12.3). Afterwards, we discuss how to combine leg odometry with diferent sensor modalities using factor graphs (Section 12.4). We conclude the chapter with a review of the open problems in the field (Section 12.5), as well as a discussion of new trends that have arisen to address them (Section 12.6).

## 12.1 Historical Background and Preliminaries

While the majority of modern legged platforms carry sensors we have studied in other chapters, e.g., cameras (Chapter 7), LiDAR (Chapter 8), and inertial measurement units (Chapter 11), in legged platforms we can additionally exploit kinematic and dynamic information from the robot’s legs to obtain measurements of the relative motion of the robot’s body, a technique known as leg odometry. The term was introduced in analogy with the odometry of wheeled vehicles, which infer the distance traveled by measuring how much the wheels turn over time.

In contrast to wheeled vehicles, legged robots move by making and breaking contacts between their legs and the ground. Each leg stride is defined by a phase where the leg is temporarily lifted of the ground (aerial or swing phase), and then it stays in non-slipping contact with the ground (stance phase). Because the legs in the stance phase are the ones responsible for propelling the robot, the leg odometry problem can be decomposed into two sub-problems:

1 Contact estimation—establishing what legs are in stance phase at a given period of time.

2 Motion estimation—determining the incremental motion from such legs during the stance phase.

Before delving into leg odometry, we briefly review historical background and preliminary concepts in this section.

## 12.1.1 Historical Background

Leg odometry has been directly related to the development of machines able to walk. The origins of legged robotics go back to 1950s and 1960s, with the first attempts to create transportation systems able to overcome the limitations of wheeled vehicles on rough terrain [674]. General Electric’s Walking Truck [776], a 1400 kg machine, is one of the best examples of the human-operated machines designed in this period. With the development of new control strategies in the 1960s, the eforts shifted to smaller scale platforms that could generate automatic walking behavior [750, 751, 1100]. Raibert’s monopods, bipeds, and quadrupeds developed in the late 1980s showed remarkable locomotion capabilities [902], motivating the use of legged platforms for real-world tasks. Achieving real-world autonomy has then been the main driver to develop leg odometry and state estimation systems.

The work by Roston and Krotkov [947] presented one of the earliest uses of leg kinematics for the estimation of the motion of a legged platform —the Ambler hexapod, a 2.5 tonnes robot designed for space exploration. Similar approaches were developed for other hexapods with diferent leg morphologies, such as RHex [669]. Later works combined leg odometry with other sensor modalities such as IMU and vision, via particle filters [208] or Kalman filters [202, 926]. The milestone work by Bloesch et al. [88] established the foundations for filtering-based, all terrain, kinematic-inertial odometry estimators for quadrupeds [89], and paved the way for extensions to bipedal platforms [948, 874].

The DARPA Robotics Challenge (DRC), developed between 2012 and 2015, motivated the development of whole-body state estimation systems for humanoid robots, aiming not only to estimate a 6-DoF pose but also the full state of the robot’s body. For this, various teams combined kinematic and dynamic information [1203] with inertial and joint sensing, as well as exteroceptive modalities such as LiDAR [315] and stereo vision [316]. In this period, most of the solutions relied on Kalman filtering but a few works also explored optimization-based approaches [1204] and factor graphs [336].

With the recent rise and commercialization of legged platforms, particularly quadrupeds, there has been a growing interest in developing more principled and resilient leg odometry and state estimation algorithms. This has been reflected in the further development of factor-graph-based estimation solutions for quadrupedal [1191] and bipedal platforms [439], which have enabled its principled fusion with other sensor modalities. Other directions have explored more fundamental challenges in modelling [19], as well as invariant estimation frameworks [441, 438]. The recent DARPA Subterranean (SubT) Challenge (2018-2021) also posed new challenges for legged state estimation in extreme scenarios, where complementary estimation solutions leveraged diferent sensor modalities across diverse legged, wheeled, and aerial platforms [558, 1287, 838]. Section 12.6 will provide further insights on the recent trends, and how they are re-shaping the research in state estimation for legged platforms.

## 12.1.2 Reference Frames

Figure 12.1 illustrates the reference frames relevant to our estimation problem. The inertial frame ${ \mathcal { F } } ^ { w }$ and the base frame $\mathcal { F } ^ { b }$ are rigidly attached to the ground and the robot’s floating base, respectively. Without loss of generality, we assume the IMU to be coincident with $\mathcal { F } ^ { b }$ . A frame is attached to each end efector, corresponding to the feet $( \mathcal { F } ^ { f 1 }$ and $\mathcal { F } ^ { f 2 }$ in the example shown in Figure 12.1).

Additionally, one or more temporary inertial frames $\mathcal { F } ^ { k }$ are created when a foot comes into contact with the ground. These are coincident with the foot frame at touchdown for humanoids, or have the same Cartesian position for quadrupeds with point feet.

## 12.1.3 State Definition

The state of a legged robot is defined by the pose and velocity of its base, as well as the joint states. However, in this chapter we assume the joint states to be measured directly by dedicated sensors (Section 12.1.6), leaving only the pose and velocity of the robot’s base as the objective of our estimation. Therefore, we use the term state estimation and odometry interchangeably, with the latter term being equivalent to SLAM without loop closures [136].

More formally, the robot state is defined as the set combining position, orienta-

![](images/33f8b75f42cd87fe5900c12578c8a4d4836f6f50a891b7cf38613d4a225244d2.jpg)  
Figure 12.1 Reference frame conventions for legged robots. The world frame ${ \mathcal { F } } ^ { w }$ is fixed to earth, while the base frame $\mathcal { F } ^ { b }$ is attached to the main chassis. Without loss of generality, the IMU frame is not shown as it can be considered coincident with $\mathcal { F } ^ { b }$ . When a foot touches the ground, a contact frame $\mathcal { F } ^ { k }$ is defined. $\mathcal { F } ^ { k }$ is rigidly attached to earth, perpendicular to the ground, and coincident with the foot frame $\bar { \mathcal { F } } ^ { f 2 }$

tion, linear velocity, and angular velocity:

$$
\boldsymbol {x} _ {k} = \left[ \begin{array}{c c c c} \boldsymbol {t} _ {k} & \boldsymbol {R} _ {k} & \boldsymbol {v} _ {k} & \boldsymbol {\omega} _ {k} \end{array} \right] ^ {\mathsf {T}}\tag{12.1}
$$

where the following conventions are adopted: the robot position $t = t _ { b } ^ { w } \in \mathbb { R } ^ { 3 }$ and orientation $\pmb { R } = \pmb { R } _ { b } ^ { w } \in \mathrm { S O } ( 3 )$ express the pose of the robot’s base in world coordinates; the robot velocities $\pmb { v } = \pmb { v } _ { b } ^ { b } , \ \pmb { \omega } = \pmb { \omega } _ { b } ^ { b } \in \mathbb { R } ^ { 3 }$ express the base’s twist, in base coordinates.

## 12.1.4 Legged Robot Kinematics

A legged robot is kinematically described by a main link for the body (also called trunk, or torso) to which one or more kinematic chains (i.e., the legs) are attached. In this chapter, we consider the most common kinematic configurations adopted in practice: bipeds with 6 actuated DoF per leg and flat feet, and quadrupeds with 3 DoF per each leg and point feet (Figure 12.2). We assume that the robot has a rigid body $( e . g .$ , with no articulated spine), and ignore any other upper limbs $( e . g .$ • arms).

For leg odometry, we are interested in modeling the relative pose (or position) of the flat (or point) feet with respect to the robot’s body. Let $\pmb q \in \mathbb { R } ^ { N } = [ q _ { 1 } , . . . , q _ { N } ] ^ { \mathsf { T } }$ be the set of joint positions of an articulated robot with N active DoFs, which correspond to the angular position of revolute joints of the legs. In Figure 12.1 and for the rest of the chapter, the number of active DoFs of the quadruped and biped is $N = 1 2$ , although this can be diferent in other platforms. The joint positions q are typically measured directly via rotary encoders placed on each joint (see Section 12.1.6.1). Alternatively, they can be measured indirectly using the kinematic model of the robot and the readings from encoders placed on a transmission between the motor and the joint $( e . g .$ , by measuring the displacement of a hydraulic piston via a linear encoder, and calculating the corresponding angle of the revolute joint moved by the piston).

![](images/f548c808678327eb1afac59f9d60b723f32158c41c3dcacfd1552d0e799f5fa2.jpg)  
Figure 12.2 Kinematic chains of typical legged robots: quadrupeds and humanoids.

The time derivatives of the joint positions are the joint velocities $\dot { \pmb q } = [ \dot { q } _ { 1 } , \dots , \dot { q } _ { N } ] ^ { \mathsf { T } }$ which are typically estimated by numerical diferentiation of the encoder readings. In some cases, when the reading are particularly noisy, additional sensing such as IMUs placed at the links can also be used to estimate the joint velocities [1205].

Given ${ \mathbf { } } q ,$ for each foot $f$ we define the corresponding forward kinematics function $\mathbf { f k } ( \pmb q ) : \mathbb { R } ^ { 1 2 }  \mathrm { S E } ( 3 )$ [717], which maps the joint positions to the pose of the foot with respect to the robot’s base:

$$
\boldsymbol {T} _ {f} ^ {b} = \mathbf {f k} (\boldsymbol {q}) = \left[ \begin{array}{c c} \mathbf {f} _ {R} (\boldsymbol {q}) & \mathbf {f} _ {p} (\boldsymbol {q}) \\ \mathbf {0} & 1 \end{array} \right]\tag{12.2}
$$

where we also define two convenient functions $\mathbf { f } _ { R } : \mathbb { R } ^ { 1 2 }  \mathrm { S O } ( 3 )$ and $\mathbf { f } _ { p } : \mathbb { R } ^ { 1 2 } \to \mathbb { R } ^ { 3 }$ expressing the orientation and Cartesian position of the foot in the base frame, respectively. For quadruped robots with point feet, only ${ \bf f } _ { p } ( { q } )$ is used for leg odometry, since the foot can pivot on the contact point with no change in the joint positions.

The time derivative of the forward kinematics function of foot $f$ from (12.2) is the Jacobian matrix $\pmb { J } ( \pmb q ) : \mathbb { R } ^ { 1 2 }  \mathbb { R } ^ { 6 \times 1 2 }$ which can be used to compute the velocity of a single foot with respect to the robot’s base as follows [718]:

$$
\left[ \begin{array}{c} \boldsymbol {v} _ {f} ^ {b} \\ \boldsymbol {\omega} _ {f} ^ {b} \end{array} \right] = \boldsymbol {J} (\boldsymbol {q}) \dot {\boldsymbol {q}} = \left[ \begin{array}{c} \boldsymbol {J} _ {v} (\boldsymbol {q}) \\ \boldsymbol {J} _ {\omega} (\boldsymbol {q}) \end{array} \right] \dot {\boldsymbol {q}}\tag{12.3}
$$

where $J _ { v } ( q ) \in \mathbb { R } ^ { 3 \times 1 2 }$ and $J _ { \omega } ( \pmb q ) \in \mathbb { R } ^ { 3 \times 1 2 }$ are the linear and angular part of the Jacobian, respectively (note that some references [319][441] define the angular block first). Since $\pmb q$ expresses the position of all $N = 1 2$ joints, but each leg is kinematically independent from the others, $\pmb { J } ( \pmb q )$ is as a sparse block matrix, where we indicate the non-zero block as $\bar { J } ( q )$ . These can be used to map the subset of joint angle velocities of a specific leg to the linear and angular velocity of the corresponding foot $f .$ For example, the Jacobian of the second leg of a humanoid $J _ { 2 } ( \pmb q )$ is represented as:

$$
\boldsymbol {J} _ {2} (\boldsymbol {q}) = \left[ \begin{array}{c c} \mathbf {0} _ {6} & \bar {\boldsymbol {J}} _ {2} (\boldsymbol {q}) \end{array} \right] = \left[ \begin{array}{c} \boldsymbol {J} _ {2, v} (\boldsymbol {q}) \\ \boldsymbol {J} _ {2, \omega} (\boldsymbol {q}) \end{array} \right] = \left[ \begin{array}{c c} \mathbf {0} _ {3 \times 6} & \bar {\boldsymbol {J}} _ {2, v} (\boldsymbol {q}) \\ \mathbf {0} _ {3 \times 6} & \bar {\boldsymbol {J}} _ {2, \omega} (\boldsymbol {q}) \end{array} \right]\tag{12.4}
$$

where $\bar { J } _ { 2 } ( \pmb q ) \in \mathbb { R } ^ { 6 \times 6 }$ indicates the non-zero block of the Jacobian matrix.

The expressions from (12.2) to (12.3) are the basis to design state estimators for legged platforms, as they relate the joint states to the robot’s base motion. However, as mentioned previously, we assume we can measure the joint states directly via encoders, as explained in Section 12.1.6.

## 12.1.5 Legged Robot Dynamics

The dynamics of a floating-base articulated-body system can be expressed as two coupled dynamics equations, computed using Recursive Newton-Euler algorithms [319]. The first equation describes the dynamics of the floating-base body (6 DoF, underactuated), while the second describes the dynamics of the N rigid-bodies (i.e., $N = 1 2 )$ attached to it through active joints $( i . e . ,$ active DoF). The two equations of motion can be put in matrix form as follows:

$$
\boldsymbol {M} (\boldsymbol {q}) \left[ \begin{array}{c} \dot {\boldsymbol {v}} \\ \dot {\boldsymbol {\omega}} \\ \ddot {\boldsymbol {q}} \end{array} \right] + \boldsymbol {h} (\boldsymbol {q}, \dot {\boldsymbol {q}}) = \left[ \begin{array}{c} \boldsymbol {J} _ {b} ^ {\mathsf {T}} \\ \boldsymbol {J} _ {q} ^ {\mathsf {T}} \end{array} \right] \boldsymbol {f} + \left[ \begin{array}{c} \boldsymbol {0} _ {6} \\ \boldsymbol {\tau} \end{array} \right]\tag{12.5}
$$

where the first term $M ( q )$ is the mass matrix, which is multiplied by the stack of $\dot { \pmb v } \in \mathbb { R } ^ { 3 } , \dot { \pmb \omega } \in \mathbb { R } ^ { 3 }$ , and $\ddot { \pmb q } \in \mathbb { R } ^ { 1 2 }$ which represent the floating-base linear, floating-base angular, and active joints accelerations, respectively. The second term $\pmb { h } \in \mathbb { R } ^ { 1 8 }$ is a bias term that accounts for Coriolis, centrifugal, and gravitational efects. Regarding the right side of (12.5), the last term describes the torques of the base, which are zero because the base is not actuated, and the torques of the active joints $\tau \in \mathbb { R } ^ { 1 2 }$

Finally, the second to last term is the most relevant for leg odometry and its factors have variable dimensions depending on the robot configuration (humanoid or quadruped) and the number of legs in contact. Let c be the number of legs in contact and d be the number of active joints per a single leg $( d = 3$ for quadrupeds, $d = 6$ for humanoids). Then, $J _ { b } \in \mathbb { R } ^ { d c \times 6 }$ is the Jacobian matrix mapping the base twist to feet velocities, which depends on the forward kinematics of the robot and its absolute body orientation in the inertial frame $\mathcal { F } ^ { w } . \ J _ { q } \in \mathbb { R } ^ { d c \times 1 2 }$ is the stack of Jacobians described in (12.3) whose arrangement depends on the type of robot and number of contact legs. For example, a humanoid standing on both legs will have:

$$
\boldsymbol {J} _ {q} = \left[ \begin{array}{c} \boldsymbol {J} _ {1} (\boldsymbol {q}) \\ \boldsymbol {J} _ {2} (\boldsymbol {q}) \end{array} \right]\tag{12.6}
$$

For quadrupeds, since the leg can pivot around the contact point, only the linear velocity Jacobian $J _ { v }$ is used for $J _ { q }$ . For example, a quadruped standing on the first, third, and fourth leg, will have:

$$
\boldsymbol {J} _ {q} = \left[ \begin{array}{c} \boldsymbol {J} _ {1, v} (\boldsymbol {q}) \\ \boldsymbol {J} _ {3, v} (\boldsymbol {q}) \\ \boldsymbol {J} _ {4, v} (\boldsymbol {q}) \end{array} \right]\tag{12.7}
$$

The last term to consider is $\textbf { \textit { f } } \in \mathbb { R } ^ { d c }$ , which represents the collection of all the forces and $/ \mathrm { o r }$ torques acting at each foot in contact with the ground. For a quadruped with all feet on the ground, $f$ is the stack of four linear forces:

$$
\boldsymbol {f} = \left[ \begin{array}{c} \boldsymbol {f} _ {1} \\ \boldsymbol {f} _ {2} \\ \boldsymbol {f} _ {3} \\ \boldsymbol {f} _ {4} \end{array} \right].\tag{12.8}
$$

For a humanoid with both feet on the ground, it contains the linear forces and torques acting on the three axes of the contact point:

$$
\boldsymbol {f} = \left[ \begin{array}{c} \boldsymbol {f} _ {1} \\ \boldsymbol {\tau} _ {1} \\ \boldsymbol {f} _ {2} \\ \boldsymbol {\tau} _ {2} \end{array} \right].\tag{12.9}
$$

As explained in more detail in Section 12.3.4, we can infer leg contact using the aforementioned forces and torques.

## 12.1.6 Joint Sensing

Similarly to fixed-base manipulators, the joints and the end efectors of legged robots are equipped with a variety of sensors, which are primarily used for planning and control [1012]. We briefly describe the most important sensors that are used also for leg odometry, namely encoders, force/torque sensors, and contact sensors.

## 12.1.6.1 Rotary Encoders

Rotary encoders are electro-mechanical devices that convert an angular position of a rotating shaft into an analog or digital signal. In legged robots, they enable us to measure the joint angles and determine the robot kinematics, but they can also be found in other components. For example, mechanical LiDARs use them to measure the azimuthal angle of the beam array (see Chapter 8).

Encoders can be categorized depending on the principle of operation (optical or magnetic), the type of reading (absolute or incremental), and type of output (analog or digital). The most adopted type on legged robots are absolute and relative optical digital encoders; other encoders are discussed in more detail in related literature [719].

![](images/f046799f8603426a2c19af2417d07e52166364482e656eb4c75a213b20a4e8bd.jpg)  
Figure 12.3 (a) Principle of operation of an 8-bit optical absolute encoder. An IR light beam hits a rotating disc that masks light according to a specific pattern that encodes the angle of rotation of the disc. An array of photoresistors convert the absence or presence of the light in each sector to a binary number encoded with a Gray code. The Gray code is then translated into a decimal number representing the absolute angle of rotation. The encoder in this example has a resolution of 360/256 = 1.41 degrees. (b) Principle of operation of an optical incremental encoder. The two photoresistors, A and B, are placed with a 90 degrees phase shift. A rising edge on A followed by a falling edge on B indicates a clockwise rotation. The change from AB = 11 to AB = 10 causes the increment of the counter.

Absolute optical digital encoders (Figure 12.3a) measure the joint angles in an absolute manner —for the same joint configuration they will provide the same sensor readings. Their operation principle is that an IR light source (e.g., a Light Emitting Diode (LED)) hits an array of sensitive elements (e.g., photoresistors) disposed radially on the static part of the device. In between the light source and the sensitive elements sits a disc that rotates with the shaft. The disc is divided in concentric sectors that can be either opaque or transparent. The sectors are arranged according to a pattern that encodes a specific angular range to a binary number. The binary number is ordered according to the Gray code, which maps consecutive natural numbers to binary numbers that always difer by only one bit, which reduces chances of reading errors. The process is illustrated in Figure 12.3a for an 8-bit encoder. The angular resolution of the device is determined by the number of bits (i.e., the number of concentric sectors) used to make the binary word encoding the angle. For instance, for an 8-bit sensor there are 256 possible values, and then the angular resolution is 360/256 = 1.41 degrees.

Incremental optical encoders (Figure 12.3b), conversely, measure relative angular changes with respect to the initial configuration, i.e., they measure a zero angle when they are turned on, and then measure the angle relative to that reference point. The angle is calculated by adding or subtracting small angle increments, depending on the direction of rotation. Instead of relying on Gray codes, they operate by using a simpler codewheel made of an opaque material with regular slots placed radially, such that a single photoresistor A produces a square wave over time when the disc rotates at constant speed. A second photoresistor B is placed at 90 degrees out of phase with the first one. The 2-bit word composing the two signals AB can have four diferent values at any given time, and the transition between them is used to determine the direction of rotation and whether the count has to be increased or decreased [719]. Because of their simpler construction and lower cost, high resolution incremental encoder have been used to compute the joint angle after a lower resolution absolute encoder measured the initial angle [987]. Even though they are still in use, incremental encoders are being rapidly replaced by absolute encoders, whose technology development improves their resolution while reducing their cost.

## 12.1.6.2 Force and Torque Sensors

Force and torque sensors are devices that convert a linear force (applied to a point on a surface) or a mechanical torque (applied to a shaft) into an electrical signal. They are primarily used for torque control on the actuators or to sense the interaction between the end efector and the environment. In legged locomotion, each step involves forces being applied to the ground that need to be measured (directly or indirectly) so that stance legs can be identified.

The principle of operation for both type of quantities (force and torque) is typically the same, with diferent geometries: the internal surfaces of the sensors are shaped in a way that would slightly deform under stress along the direction where the force needs to be measured. Glued to those surfaces is a flexible variable resistive element, the strain gauge. The electrical resistance of the strain gauge changes proportionally to the amount of deformation it sustains, with compression (extension) causing a reduction (increase) in resistivity. Figure 12.4 shows an example with strain gauges applied to a load cell to measure linear force [719]. To measure torque, a series of strain gauges is applied to flexible spokes of a wheel connected to a motor shaft.

To measure all forces and torques acting on an end efector, 6-axis sensors are available, containing strain gauges in a number of configurations suficient to measure forces and torques in all directions. These are commonly used for manipulation tasks, but can be also found on humanoids feet to directly measure the interaction with the ground.

With the introduction of cost efective dynamic legged robots, which was made possible by a backdriveable motor design [544], torques can be estimated from the motor currents, which are proportional to the torque by a constant factor.

## 12.1.6.3 Contact Sensors

Since the main use for force and torque sensors in leg odometry is to determine the stance legs, alternative cost efective solutions are contact sensors, whose output is a binary number indicating whether a certain foot is in contact or not. This type of sensor was mainly developed for small to medium quadruped robots, whose feet consist of a spherical or circular rubber sole.

![](images/b9d0dc56c07aa9348f4ad83ec3bbe1dd012a6d49bf6b70fbd2a96e3dc5a4e7e8.jpg)  
Figure 12.4 Principle of operation of force and torque sensors. A strain gauge is applied on compressible or flexible elements when under load. Inside the loadcell (on the left), a strain gauge is applied such that a compression would be detected as a reduction in resistivity. Inside the torque sensor (on the right), several strain gauges are applied at the flexible elements (the spokes of a wheel). When torque is applied, the elements would deform by flexion. The presence of multiple strain gauges allow to work out the magnitude and direction of the torque e.g., by sensing a compression on one side of the spoke and an elongation on the other side.

The main types of contact sensors are optical [397] or mechanical [794]. Optical contact sensors are conceptually similar to encoders: a LED-photodiode pair sense light through a small aperture. When the foot is in contact, the surface of the foot deforms enough to create a displacement of a masking panel that occludes the aperture, allowing the system to detect the contact. Mechanical contact sensors use a simple pushbutton switch hidden inside the sole that is pressed when enough force is exerted on the foot.

The main disadvantage of contact sensors is the relatively slow response time compared to costly force/torque sensors. In addition, they sufer from the same drawbacks of F/T sensors: they require to route cables up to the foot and they are at risk of damage due to the main impacts they have to sustain. For these reasons, they are mostly available only for small sized quadrupeds mostly designed for indoor operations.

## 12.2 Motion Estimation

Given the joint states and kinematics of the robot, we are now interested in computing the incremental motion of the robot’s base. This can be mainly done in two ways: using the forward kinematics to obtain a relative pose between two time instants, or using the diferential kinematics to estimate the robot’s velocity instantaneously. In both cases, the underlying assumption is that a newly formed contact frame remains stationary for a certain amount of time.

![](images/4107b204f62579706266b400cf70625043908720f8f9a09df34c64abddd9bb12.jpg)  
Figure 12.5 How leg odometry works with ideal contact. Left: Assuming the contact frame $\mathcal { F } ^ { k }$ is rigidly attached to the ground (in yellow), we can determine the relative motion of the robot’s body. Right: Alternatively, we can represent how the leg moves with respect to the body frame (yellow) in two consecutive instants.

## 12.2.1 Relative Pose Estimation

Figure 12.5 shows a simplified example of a humanoid robot walking along the zxplane. The robot’s base is represented at two consecutive time instants with the frames $\mathcal { F } ^ { b }$ and $\mathcal { F } ^ { b ^ { \prime } }$ ; the foot frames and the joint positions at the same two times are defined similarly. Because the contact frame is stationary, when the foot frame and the contact frame coincide, the amount of displacement the robot’s body experiences while moving forward is the same as the foot experiences moving backwards from the robot’s body:

$$
\pmb {T} _ {b ^ {\prime}} ^ {b} = \pmb {T} _ {k} ^ {b} (\pmb {T} _ {k} ^ {b ^ {\prime}}) ^ {- 1} = (\pmb {T} _ {f ^ {\prime}} ^ {f}) ^ {- 1} = (\pmb {T} _ {f ^ {\prime}} ^ {b}) ^ {- 1} \pmb {T} _ {f} ^ {b} = \mathbf {f k} (\pmb {q} ^ {\prime}) ^ {- 1} \mathbf {f k} (\pmb {q})\tag{12.10}
$$

Equation (12.10) creates a mapping between the joint states and the relative pose of the robot. While a concatenation of these relative poses would efectively provide a valid motion estimate by dead reckoning from joint sensing only [947], this is only possible during the stance phase. Further, to be applied on robots with point feet, this requires at least three feet in contact with the ground at all times, making it impractical for quadrupedal platforms.

To overcome these issues, the standard approach for quadrupeds [88] is to augment the state in (12.1) with the positions $\pmb { c } _ { i } = \pmb { t } _ { k } ^ { w } \in \mathbb { R } ^ { 3 }$ of the contact frames expressed in world coordinates and associated to each leg of the robot. For humanoids, since they have ankles, the orientation of the contact points $B _ { i } = R _ { k } ^ { w } \in \mathrm { S O } ( 3 )$ can also be added to the state [948].

Further, since there is no guarantee that at any given time there are a suficient number of legs in contact $( e . g . , \mathrm { ~ a ~ }$ gallop gait has phases were all the legs are of the ground), it is also commonly assumed that an IMU is present, so the angular velocity ω in (12.1) is disregarded, and the IMU biases are included as part of the state instead (see Chapter 11).

With all the previous considerations, the corresponding states of interest for quadrupeds and bipeds are then defined as:

$$
\boldsymbol {x} _ {k} = \left[ \begin{array}{c c c c c c c c c} \boldsymbol {t} _ {k} & \boldsymbol {R} _ {k} & \boldsymbol {v} _ {k} & \mathbf {b} _ {k} ^ {a} & \mathbf {b} _ {k} ^ {\omega} & \boldsymbol {c} _ {1} & \boldsymbol {c} _ {2} & \boldsymbol {c} _ {3} & \boldsymbol {c} _ {4} \end{array} \right] ^ {\mathsf {T}}\tag{12.11}
$$

$$
\boldsymbol {x} _ {k} = \left[ \begin{array}{c c c c c c c c c} \boldsymbol {t} _ {k} & \boldsymbol {R} _ {k} & \boldsymbol {v} _ {k} & \mathbf {b} _ {k} ^ {a} & \mathbf {b} _ {k} ^ {\omega} & \boldsymbol {c} _ {1} & \boldsymbol {c} _ {2} & \boldsymbol {B} _ {1} & \boldsymbol {B} _ {2} \end{array} \right] ^ {\mathsf {T}}\tag{12.12}
$$

where (12.11) represents the state of a quadruped robot, whilst (12.12) corresponds to a humanoid robot. This enables us to precisely express the motion estimate relationship from (12.10) for an arbitrary i-th leg:

$$
\pmb {T} _ {k} ^ {b} = \mathbf {f k} (\pmb {q}) = (\pmb {T}) ^ {- 1} \pmb {C} _ {i}\tag{12.13}
$$

where

$$
\boldsymbol {T} = \left[ \begin{array}{c c} \boldsymbol {R} & \boldsymbol {t} \\ \boldsymbol {0} & 1 \end{array} \right]\tag{12.14}
$$

is the pose of the base in the fixed frame, whereas

$$
\boldsymbol {C} _ {i} = \left[ \begin{array}{c c} \boldsymbol {B} _ {i} & \boldsymbol {c} _ {i} \\ \boldsymbol {0} & 1 \end{array} \right]\tag{12.15}
$$

is the pose of the foot contact in the fixed frame. Expanding (12.13) leads to:

$$
(T) ^ {- 1} C _ {i} = \left[ \begin{array}{c c} R ^ {\top} & - R ^ {\top} t \\ 0 & 1 \end{array} \right] \left[ \begin{array}{c c} B _ {i} & c _ {i} \\ 0 & 1 \end{array} \right] = \left[ \begin{array}{c c} R ^ {\top} B _ {i} & R ^ {\top} c _ {i} - R ^ {\top} t \\ 0 & 1 \end{array} \right]\tag{12.16}
$$

Rearranging (12.16), we can define the following components corresponding to the upper blocks of right-hand side matrix:

$$
\mathbf {f} _ {p} (\boldsymbol {q}) = \boldsymbol {R} ^ {\mathsf {T}} (\boldsymbol {c} _ {i} - \boldsymbol {t})\tag{12.17}
$$

$$
\mathbf {f} _ {R} (\pmb {q}) = \pmb {R} ^ {\mathsf {T}} \pmb {B} _ {i}\tag{12.18}
$$

where ${ \bf f } _ { p } ( { q } )$ denotes the relative position change of the foot in the fixed frame, and ${ \bf f } _ { R } ( { \pmb q } )$ the relative orientation change, as a function of the joint angles. Please note that for quadrupeds we only use (12.17), since we cannot obtain an orientation estimate from point feet.

Equations (12.17) and (12.18) are the basic leg odometry expressions used as measurements within estimation frameworks such as filtering or factor graphs. These will be further described in Section 12.4.

## 12.2.2 Velocity Estimation

The diferential kinematics function from (12.3) can be used to get a direct velocity measurement from each stance leg. This approach is widely adopted on quadrupeds [89, 143, 578] and less frequently on bipeds [1085]. The advantages are that velocity measurements can be easily (pre)integrated into a filter (or factor); they do not retain any history to avoid position error build up [315] and they do not need to keep track of extra states (contact poses or positions). However, we must point out that the joint velocities are usually numerically diferentiated from joint positions, possibly degrading the estimation performance due to rounding errors.

Following a similar procedure as with the relative pose measurements, we aim to describe the velocity relationships that hold while a leg in rigid contact with the ground moves. First, we observe that the contact point k must be stationary for stance legs, hence the velocity of the contact point seen from the fixed frame must be zero:

$$
\pmb {v} _ {k} ^ {w} = \mathbf {0}.\tag{12.19}
$$

Furthermore, the velocity of the contact point k must coincide with the velocity of the foot $f ,$ also described by the Jacobian matrix (12.3):

$$
\boldsymbol {v} _ {k} ^ {b} = \boldsymbol {v} _ {f} ^ {b} = \boldsymbol {J} _ {v} (\boldsymbol {q}) \dot {\boldsymbol {q}}.\tag{12.20}
$$

Since the angular velocity of the robot is measured by the IMU, we focus on the linear velocity only. From (12.19) and (12.20), we can determine the robot’s body velocity as:

$$
\begin{array}{c} \boldsymbol {v} _ {k} ^ {w} = \boldsymbol {v} _ {b} ^ {w} + \boldsymbol {\omega} _ {b} ^ {w} \times \boldsymbol {t} _ {k} ^ {b} + \boldsymbol {v} _ {k} ^ {b} \\ \boldsymbol {0} = \boldsymbol {v} _ {b} ^ {w} + \boldsymbol {\omega} _ {b} ^ {w} \times \mathbf {f} _ {p} (\boldsymbol {q}) + \boldsymbol {J} _ {v} (\boldsymbol {q}) \dot {\boldsymbol {q}} \\ \boldsymbol {v} _ {b} ^ {w} = - \boldsymbol {\omega} _ {b} ^ {w} \times \mathbf {f} _ {p} (\boldsymbol {q}) - \boldsymbol {J} _ {v} (\boldsymbol {q}) \dot {\boldsymbol {q}} \end{array}\tag{12.21}
$$

where we take into account the additional linear velocity caused by the lever arm between the base and the foot [270]. (12.21) maps the absolute velocity of the robot to the forward and diferential kinematics functions and can therefore be used as a measurement update or factor in a graph, as we will describe in Section 12.4. Note that since we conventionally express velocities in body coordinates, these can be obtained by using the robot orientation $R = R _ { b } ^ { w }$

Up to now we assumed that the stance legs are known. In the next section, we describe diferent methods to identify them.

## 12.3 Contact Estimation

The definition of contact estimation varies with the application. For example, in collaborative robotics, the end efector of a manipulator might be considered in contact as soon as it is “touching” something, i.e., there is a non negligible external force exerted on it. For leg odometry, a foot can be considered in contact only when the contact point is stationary over time; on robots with point feet, this means ensuring it does not slip.

![](images/c18acbe52212970a57b17662f23ebebd89dd2a58c262a9e4389f2370bba61785.jpg)  
Figure 12.6 The contact point on a quadruped’s leg. A leg can be considered in stance when the force $\mathbf { \bar { \mathbf { f } } } = [ f _ { x } , \bar { f _ { y } } , f _ { z } ] ^ { \top }$ applied by the foot stays within the friction cone.

Technically, a foot does not slip when the vertical component of the Ground Reaction Force (GRF), $f _ { z }$ , is within the friction cone [949]:

$$
\sqrt {f _ {x} ^ {2} + f _ {y} ^ {2}} \leq \mu_ {x, y} f _ {z}\tag{12.22}
$$

where $f _ { x }$ and $f _ { y }$ are the tangential components of the GRF with respect to the contact plane, which depend on the local morphology of the terrain. $\mu _ { x , y }$ is the friction coeficient, which depends on the mechanical properties of the ground and the foot touching it.

Humanoids with flat feet can also exert a torque on the ground. This introduces an additional condition to the non-slipping condition (12.22), which requires tha the foot does not rotate:

$$
\left[ \begin{array}{c} - \tau_ {y} / f _ {z} \\ \tau_ {x} / f _ {z} \end{array} \right] \leq \left[ \begin{array}{c} C o P _ {x} \\ C o P _ {y} \end{array} \right]\tag{12.23}
$$

$$
| \tau_ {z} | \leq \mu_ {z} f _ {z}\tag{12.24}
$$

where $\tau$ is the contact torque, $\mu _ { z }$ is the rotational coeficient of friction, and $C o P _ { x } ,$ $C o P _ { y }$ denote upper limits of the components of the center of pressure, which define the contact support polygon bounds that are functions of contact surface geometry.

Since a suficiently-high normal force $f _ { z }$ would guarantee that inequalities (12.22)- (12.24) are satisfied regardless of the other contact wrench (force and torque) dimensions, the most adopted approach for contact estimation is to simply threshold $f _ { z }$ Then, the only diferences from an implementation point of view are how the force is measured/estimated (i.e., contact sensors, $\mathrm { F / T }$ sensors, joint sensing, IMUs), and the specific characteristics of the robot.

## 12.3.1 With Contact Sensors

Contact sensors implicitly threshold $f _ { z }$ in hardware, as they are tuned such that the binary signal they provided is only activated when the measured force exceeds the nominal $f _ { z }$ . This is the simplest case, as the leg odometry can directly rely on the binary state provided by these sensors.

## 12.3.2 With Force/Torque Sensors

When $\mathrm { F / T }$ sensors are present on the foot, $f _ { z }$ can be measured directly over time. This permits to associate specific force patterns to events that are not just binary. For example, a small but rising force that lasts for more than a certain time is an indication that the foot is striking the ground but not yet in stance. Conversely, a force that falls below a certain value $( e . g .$ , half of the expected load for one leg) means the foot is about to break the contact and it is therefore not reliable. In both cases, the information coming from that leg need to be discarded or its associated uncertainty increased [315].

## 12.3.3 With IMUs

As seen in Chapter 11, IMUs are inexpensive sensors that provide acceleration and rotational velocity measurements. While we typically use them to measure these quantities with respect to the robot’s body, we can also use them on the legs or feet. Since any force applied to the foot $( e . g .$ , during a touch down) would cause a change to its acceleration, some works used them to implicitly detect the stance legs [1230, 949, 729]. The main advantage of this approach is that the sensor is not sustaining an impact directly, so it is less likely to break, at the cost of additional signal processing to efectively detect such acceleration changes.

## 12.3.4 From Joint Torque Sensing

While it might seem straightforward to add additional sensors at the feet to detect contact, the diferent robot morphologies, design, and integration challenges might not make it always possible. In this case, the GRF can be estimated from the joint torques by exploiting the robots’ dynamics (see (12.1.5)).

Using a quadruped platform as an example, we can exploit the block-wise structure of $J _ { q }$ to compute the force at the end efector from (12.5) as:

$$
\boldsymbol {f} _ {i} = - (\bar {\boldsymbol {J}} _ {i, v} ^ {\top}) ^ {- 1} \big (\boldsymbol {\tau} _ {i} - \boldsymbol {h} _ {q, i} - \boldsymbol {F} ^ {\top} \dot {\boldsymbol {v}} \big)\tag{12.25}
$$

where: $f _ { i } \in \mathbb { R } ^ { 3 }$ and $\tau _ { i } \in \mathbb { R } ^ { 3 }$ are the GRF and the torque leg $i ; \bar { J } _ { i , v }$ is the non-zero block i-th foot Jacobian $J _ { v }$ (which for quadrupeds is a square matrix); $\pmb { F } \in \mathbb { R } ^ { 3 \times 3 }$ is one of the blocks of the mass matrix; $\pmb { h } _ { i , q } \in \mathbb { R } ^ { 3 }$ is the vector of centrifugal/Cori olis/gravity torques for leg $i .$

Note that the estimate for $f _ { i }$ can only be in base coordinates. However, to recover the actual GRF there are two pieces of information missing:

the local inclination of the terrain, which the orientation of the contact force depends on. While the ankle joints of a humanoid can give a good approximation, for quadrupeds the orientation of the contact frame cannot be determined without exteroceptive sensing, but can be inferred heuristically from the other feet in contact (e.g., by fitting a plane through them); [329]

the friction coeficient, which the horizontal components of the force depend on. The friction coeficient depends on the material the robot is stepping on, so it can only be known a priori or inferred from the amount of slippage the robot is experiencing [513].

In general, establishing the contact states from joint sensing remains an open problem, and several techniques have been developed to detect contact in a probabilistic fashion, (e.g., by combining also kinematics as well as dynamics of the robot [497]) or by using learning methods (see Section 12.6).

## 12.4 Using Leg Odometry for State Estimation

Now that we have specified the main steps required to obtain leg odometry measurements, in this section we describe how to integrate them into an estimation framework to solve the state estimation problem. The predominant estimation solutions are based on filtering approaches, which combine IMU and leg odometry at the high frequency required for closed-loop control. The factor-graph-based smoothing approaches described in the previous chapters have only been adopted on legged platforms in the last few years. However, their focus has not been on providing estimates for control but rather lower frequency estimates for mapping, which benefit from slower sensors such as LiDARs or cameras.

Regardless of the method, for optimally fusing leg odometry with other sensor modalities we need to quantify its associated uncertainties. This is the first topic we will cover before presenting the filtering and smoothing approaches.

## 12.4.1 Encoder Noise Propagation

The main sources of uncertainty in leg odometry are the robot’s joints encoders, which measure the joint positions and are afected by noise. This noise can be modeled as an additive zero-mean Gaussian term $\pmb { \eta } _ { q } \in \mathcal { N } ( \mathbf { 0 } , \pmb { \Sigma } _ { q } )$ , such that the true value q and the measured value q˜ are related as follows:

$$
\tilde {\boldsymbol {q}} = \boldsymbol {q} + \boldsymbol {\eta} _ {q}.\tag{12.26}
$$

Since the forward and diferential kinematics functions involve rotations, they are therefore nonlinear and they will not preserve the Gaussian properties of the encoder noise. However, as done in previous chapters, we can consider a first-order approximation —which is locally linear and preserves Gaussianity— using the Jacobian function [439]:

$$
\mathbf {f} _ {p} (\boldsymbol {q} + \boldsymbol {\eta} _ {q}) \approx \mathbf {f} _ {p} (\boldsymbol {q}) + \boldsymbol {J} _ {c} (\boldsymbol {q}) \boldsymbol {\eta} _ {q}\tag{12.27}
$$

where $J _ { c } ( q )$ is the body manipulator Jacobian, $i . e .$ , the same as the manipulator Jacobian $\pmb { J } ( \pmb q )$ but expressed in the contact frame.

The same Gaussian assumption can also be applied to velocity measurements afected by encoder noise $\pmb { \eta } _ { q }$ and encoder velocity noise $\pmb { \eta } _ { \dot { q } }$ [1191], considering that:

$$
\boldsymbol {J} (\boldsymbol {q} + \boldsymbol {\eta} _ {q}) (\dot {\boldsymbol {q}} + \boldsymbol {\eta} _ {\dot {q}}) \approx \boldsymbol {J} (\boldsymbol {q}) \dot {\boldsymbol {q}} + \frac {\partial}{\partial \boldsymbol {q}} (\boldsymbol {J} (\boldsymbol {q}) \dot {\boldsymbol {q}}) \boldsymbol {\eta} _ {q} + \boldsymbol {J} (\boldsymbol {q}) \boldsymbol {\eta} _ {\dot {q}}.\tag{12.28}
$$

From (12.27) and (12.28), the extra terms multiplied by the noise terms can simply be grouped into a single term, since they are all linear combinations of a Gaussian term for a given encoder measurement.

## 12.4.2 Factor Graph Smoothing

To generate locomotion behaviors while avoiding falls and other catastrophic failures, legged robots have strict real-time control and high-frequency state estimation requirements. Historically, those requirements were met by using nonlinear variants of the Kalman Filter, such as the Extended Kalman Filter (EKF) [88, 143], the Unscented Kalman Filter (UKF) [89], or the Invariant-EKF [441, 1255, 670]. These types of filter would typically fuse together high-frequency sensor data, such as inertial and kinematics, to feed the controller. Exteroceptive sensor updates within the control loop have also been demonstrated [144] but those are normally relegated to mapping and planning purposes.

One limitation of Kalman filtering-based methods is that they are designed to have a process model in addition to the measurement model. When such a model is not available, it is usually replaced by a constant velocity model or, more often, IMU propagation. This suggests that factor-graph-based methods are a more general approach, as they consider both process models and measurement models in a general manner —as a relationship between states and measurements.

Factor-graph-based methods for legged systems mainly difer in the number of estimated states and the time horizon. When only two consecutive states are considered, a factor graph resembles a filter; the Two-State Implicit Filter (TSIF) [92] is an instance of this case. When the window is increased, instead of estimating only the most current state as the TSIF, the factor graph has the ability to correct a history of past states within a time window. The frequency of the states considered is a design decision: adding more frequent states at a high frequency (e.g., IMU rate) simplifies the design of the estimator but it requires to reduce the time window to keep the computational requirements bounded. Conversely, longer time horizons with a fixed number of states can be achieved by preintegrating measurements, as showed for IMU measurements in Chapter 11.

We next present two examples from the related literature that illustrate how preintegration theory and the leg odometry concepts previously introduced are leveraged in a factor graph estimation framework. We particularly focus on the case of contact preintegration for bipeds [440], and velocity bias preintegration for quadrupeds [1191]. In both cases, the measurements from Section 12.2 will be reformulated in terms of residuals and covariances for the factors of the graph.

## 12.4.2.1 Contact Preintegration

Contact preintegration aims to integrate the relative motion increments from the kinematics of a humanoid robot, and add them as factors that link two humanoid states, defined as in (12.12). This idea was presented by Hartley et al. [440], and the proposed factor graph is shown in Figure 12.7a.

The factors are generally standard: a prior factor (in black) anchors the graph, while a preintegrated IMU factor (orange) introduces the motion prior from the IMU. Additionally, for humanoids we add a forward kinematics factor (in green) that constrains the pose of the contact frames, while the contact preintegration factor (in blue) encodes the relative motion between contact states of the two legs.

Forward Kinematics Factor. The forward kinematics factor relates the pose of the contact frame at the feet to the pose of the robot, both expressed in the inertial frame, via the forward kinematics of the stance leg.

By plugging the encoder noise from (12.27) into the relative pose measurement of (12.17) and (12.18), we can define the following residual and covariance for the forward kinematics factor:

$$
\boldsymbol {r} _ {\mathcal {F}} = \operatorname{Log} \left(\boldsymbol {C} _ {i} ^ {- 1} \boldsymbol {T} \mathbf {f k} (\tilde {\boldsymbol {q}})\right)\tag{12.29}
$$

$$
\pmb {\Sigma} _ {\mathcal {F}} = \pmb {J} _ {c} (\tilde {\pmb {q}}) \pmb {\Sigma} _ {q} \pmb {J} _ {c} ^ {\mathsf {T}} (\tilde {\pmb {q}})\tag{12.30}
$$

where the residual enforces that the diference between the contact frame and the robot frame are close to the forward kinematics, given the uncertainty propagated from the encoders’ noise.

Contact Preintegration Factor. The contact preintegration factor adds an additional constraint on the contact point. Ideally, if there is no slip on the stance leg and the pose of the contact frame should remain unaltered; in practice, slip occurs and can be modeled as Gaussian noise added to the velocities of the contact point. The contact preintegration factor models how the contact point can change between two time instants due to this noise.

Technically, given two consecutive states $\mathbf { \boldsymbol { x } } _ { i }$ and $\boldsymbol { \mathscr { x } } _ { j }$ at times $t _ { i }$ and $t _ { j }$ , respectively, the following relationship holds:

$$
\Delta \tilde {\pmb {B}} _ {i j} = \pmb {B} _ {i} ^ {\mathsf {T}} \pmb {B} _ {j} \mathrm{Exp} (\delta \pmb {\theta} _ {i j}) = \mathbf {I}\tag{12.31}
$$

$$
\Delta \tilde {\pmb {c}} _ {i j} = \pmb {B} _ {i} ^ {\mathsf {T}} (\pmb {c} _ {j} - \pmb {c} _ {i}) + \delta \pmb {d} _ {i j} = \pmb {0}\tag{12.32}
$$

![](images/ee86d7d6374177c82b172a81addc80e7c8df71feb4dc03202427190b106a7bac.jpg)  
Figure 12.7 Factor graph formulations for preintegrated contact (top) and velocity (bottom). Additional measurements $( e . g .$ , from exteroceptive sensors, which constrain two states) can be easily added as additional factors (magenta).

where we have used the rotational and translational components of the contact frames $C _ { i }$ and $C _ { j }$ as stated in (12.15). The terms $\delta \pmb { \theta } _ { i j }$ and $\delta d _ { i j }$ are preintegrated contact noise terms, which are introduced to model the uncertainty on the contact point velocity as a zero-mean Gaussian variable [440]. The preintegrated contact factor is then given by the following rotation and translation residuals and covariance:

$$
\boldsymbol {r} _ {\mathcal {C}} = \left[ \begin{array}{c} \operatorname{Log} \left(\boldsymbol {B} _ {i} ^ {\mathsf {T}} \boldsymbol {B} _ {j}\right) \\ \boldsymbol {B} _ {i} ^ {\mathsf {T}} (\boldsymbol {c} _ {j} - \boldsymbol {c} _ {i}) \end{array} \right]\tag{12.33}
$$

$$
\boldsymbol {\Sigma} _ {\mathcal {C}} = \left[ \begin{array}{c c} \boldsymbol {\Sigma} _ {w} & \mathbf {0} \\ \mathbf {0} & \boldsymbol {\Sigma} _ {v} \end{array} \right] \Delta t _ {i j}\tag{12.34}
$$

where we have rearranged and stacked (12.31) and (12.32) into a single vector residual. The covariance $\Sigma _ { \mathcal { C } }$ is made of the time integration of the contact angular covariance $\Sigma _ { w }$ and linear velocity covariance $\Sigma _ { v }$ over $\Delta t _ { i j }$

As a last note, an important limitation of the contact preintegration factor is that it is only valid for the same stance leg, during the stance phase. Henceforth, it is not valid for the switching dynamics of a legged platform. This has been addressed in follow up work [439], by modeling and properly handling the contact frame switches, enabling preintegration among diferent legs.

## 12.4.2.2 Velocity Preintegration

As mentioned previously, the kinematics of point feet platforms —such as quadruped robots— cannot constrain the relative 6 DoFs between two states. This impedes the use of the forward kinematic and contact preintegration factors recently introduced. Alternatively, we can exploit the linear velocity measurements from leg odometry, reviewed in Section 12.2.2, and preintegrate them to obtain additional factors that constraint the relative change of the robot’s pose [1191, 578].

Velocity Preintegration Factor. This factor assumes that the instant linear velocity of the body can be determined from leg odometry using (12.28) and it is afected by Gaussian noise terms $\eta _ { v }$ and $\eta _ { \omega }$ :

$$
\tilde {\boldsymbol {v}} = - \boldsymbol {J} _ {v} (\boldsymbol {q}) \dot {\boldsymbol {q}} - \boldsymbol {\omega} \times \mathbf {f} _ {p} (\boldsymbol {q}) + \boldsymbol {\eta} _ {v}.\tag{12.35}
$$

Assuming the robot has constant body linear velocity between times $t _ { i }$ and $t _ { j }$ , we can preintegrate the velocity measurements to obtain:

$$
\Delta \tilde {\boldsymbol {t}} _ {i j} = \Delta \boldsymbol {t} _ {i j} + \delta \boldsymbol {p} _ {i j} = \sum_ {k = i} ^ {j - 1} \left[ \Delta \tilde {\boldsymbol {R}} _ {i k} \tilde {\boldsymbol {v}} _ {k} \Delta t \right] + \delta \boldsymbol {p} _ {i j},\tag{12.36}
$$

where, similarly to the preintegrated contact factors, $\delta \pmb { p } _ { i j }$ is a preintegrated velocity noise term [1190, 578]. Then, the preintegrated velocity factor and associated covariance are given by:

$$
\pmb {r} _ {\mathcal {V}} = \pmb {R} _ {i} ^ {\top} (\pmb {t} _ {j} - \pmb {t} _ {i}) - \Delta \pmb {t} _ {i j}\tag{12.37}
$$

$$
\boldsymbol {\Sigma} _ {\mathcal {V}, i j} = \sum_ {j - 1} ^ {k = i} \boldsymbol {\Sigma} _ {\mathcal {V}, i k} + \boldsymbol {A} \boldsymbol {\Sigma} _ {v} \boldsymbol {A} ^ {\top},\tag{12.38}
$$

with the matrix $A = \Delta \tilde { R } _ { i k } \Delta t$

## 12.4.2.3 Handling of Multiple Measurements

In the previous sections we have considered only one measurement per leg, without considering what to do when multiple legs are in contact at the same time. The presence of multiple legs in contact potentially provides redundancy and robustness, but increases the risk of inconsistencies $( e . g .$ , when diferent legs provide conflicting information). The simplest approach adopted by some works (e.g., [315]) is to pick only the leg that is deemed to be most reliable, discarding the information from the others.

Another intuitive approach is to treat each leg (and their measurements) independently. This is easier when the contact poses (or positions) are explicitly part of the state [440, 578]. When this is not the case, the velocity measurements simultaneously acquired by all legs in stance can still be treated as independent, but it is often preferable to average them into one single measurement to reduce the computational load on the filter or factor graph [1191].

## 12.4.3 Integration with Exteroceptive Sensors for SLAM

As we have seen in the previous sections, leg odometry provides an additional way to compute incremental motion between two consecutive states. Their main use is to improve the odometry estimate, such that the SLAM system building on top of it (e.g., a ) can benefit from low drift edges between nodes, which translate into less abrupt corrections during loop closures.

The integration of additional sensors, such as LiDAR and cameras, is naturally handled by both filtering and smoothing approaches by simply adding more measurements to the former and factors to the latter. There are however subtle details to be considered while doing so. Fusing measurements from multiple independent sources, each one operating at diferent frequencies, levels of noise, and failure rates, is not trivial.

If a sensor modality breaks the zero-mean Gaussian noise assumption, or fails completely, the status of the filter (or factor graph) can be compromised. For this reason, also motivated by the DARPA SubT challenge, there has been a surge in loosely coupled methods that run diferent subsystems in parallel (e.g., Visual-Inertial, Legged-Inertial, and LiDAR-Inertial) while triaging their outputs and select the best estimate from each subsystem [299, 558].

The alternative to loosely coupled methods are tightly coupled ones. In [1191] a fixed-lag smoother was used to fuse leg odometry with IMU, cameras and LiDAR in the same factor graph. In this case, to overcome the problems related to inconsistencies between the diferent types of factors or sensor failures, the triaging happens directly into the factors: if a sensor modality fails, the factor is simply not added to the graph. In addition, to handle noise that is not zero-mean Gaussian, robust cost functions can be used within factors.

## 12.5 Open Challenges

In previous sections we have covered how to generally compute leg odometry and fuse it with other sensor modalities. We made a number of assumptions that are often not valid in practice, and relaxing some of them remains an open problem. We briefly introduce some of the resulting open problems below.

## 12.5.1 Leg Deformation

The leg odometry equations we have seen throughout the chapter all assumed that the robot was a perfectly rigid body. When this assumption is not valid, leg odometry measurements will be biased, because the forward kinematics function computes the ideal position of the end efector and not the real one (see Figure 12.8). Similarly, when the contact point does not move, but forces are applied to it such that the legs bend, the joint angles change. When the problem occurs for short periods of time, detecting the impact by analyzing the force profile and rejecting the measurements during those periods is a strategy adopted in the past [143].

Leg Odometry for SLAM  
![](images/fb3860f4f3af9c839978223e71ac7d7793fd6c6d423dd57f4218829d9f5c9628.jpg)

Figure 12.8 Example of leg deformation on a quadruped. The real transformation between the robot base and the contact point is shown at the left. Since the forward kinematics assumes the robot’s legs are rigid, it incorrectly estimates and upward motion, as shown on the right.  
![](images/9f196b0cc1e804467e4f80f9292ec3c0458ae7e168ff01db6aefe42f281be799.jpg)  
Figure 12.9 Example of ground deformation with a quadruped. The robot initially touches the ground which is flat. While keeping the contact state on, the ground deforms and the foot sinks into it (left). As the joint angles change while the foot goes down, this motion is interpreted as an upward motion from the initial touchdown point (right)

Since bipeds tend to have longer legs, the problem of leg flexibility can be even worse on such platforms. One way to approach it would be to exploit the correlation between the leg load and the flexibility (intuitively, the more a leg is loaded, the more it will flex) by carefully modelling the bending properties of the robot considering its structural geometry and properties. This approach is however complex and typically does not generalize well.

Instead, the most adopted approach is to integrate additional IMU sensors located on the links and estimate the link orientation compared to the joint readings [1128].

## 12.5.2 Non-rigid Contacts and Slippage

If the robot is walking on soft or collapsible ground, the leg stretches penetrating the terrain (Figure 12.9, left). When the contact is first detected before the ground starts its deformation and since the contact point is assumed to be stationary, this is interpreted by the forward kinematics as an upward motion (Figure 12.9, right). Even if the efect is similar to leg deformation, in this case it is the assumption of zero velocity of the contact point to be broken [314], since the contact point moves downwards as the ground deforms. This problem is similar to slippage, when a foot is considered in contact with the ground but the forces it exerts on the terrain violate (12.22) and/or (12.24).

In this case, an exteroceptive sensor such as a camera is needed to make the velocity of the contact point observable in non-degenerate motions and robot configurations [1085]. The velocity of the contact can be explicitly tracked as an additional state in [1191], or as the derivative of the feet positions [578].

## 12.6 Further Readings & Recent Trends

In this last section, we focus on some of the recent trends in state estimation and legged robotics. While some of them address part of the open challenges discussed in the previous section, such as contact estimation, others also represent significant paradigm shifts in the current techniques —particularly those aided by learning algorithms.

Learning-based Contact Estimation. We discussed how contact estimation assumes rigid contact which is easily violated by situations such as slippage, soft terrain, or leg deformation. Given the challenges of accurately modeling these problems, it has been proposed to use data-driven methods for contact estimation. These approaches generally aim to learn a binary signal that determines when contact was established. This has been demonstrated in a supervised manner by learning contact classifiers [143], but also in an unsupervised fashion via clustering [948], where proprioceptive sensing (joint and inertial) provide the main signals for the models.

With the rise of deep learning methods, it has been proposed to use neural network architectures to determine the contact state of the feet. This has shown better generalization to a wider set of structured and unstructured environments [670]. Vision-based haptic sensors, which capitalize on the progress of machine learning and computer vision [641], are another direction that shows promise to improve force and contact estimates [1003].

End-to-End Learning. While high-frequency leg odometry and proprioceptive state estimation were developed to achieve closed-loop model-based locomotion control, the current progress in reinforcement learning (RL) has challenged their necessity. RL-based locomotion controllers showed that only the body velocity and orientation are required for locomotion, which can be provided explicitly by a standard proprioceptive state estimators [498], or even raw data from joint and inertial sensing [638, 763].

While the latter might question the need for state estimation, this is explained by the manner these particular RL-based controllers are trained. The training objective aims to track velocity commands, emulating the way in which the robot will be controlled by a human operator or planning system. To achieve this, the locomotion controller only needs to know the orientation of the base with respect to the gravity vector, as well as the instantaneous body velocity. As seen in Chapter 11, these quantities are fully observable from inertial data, hence can be implicitly estimated during training.

In contrast, another current trend in locomotion learning aims to achieve advanced mobility skills to navigate the world, by learning locomotion controllers that are able to traverse diferent obstacles and reach goals relative to a starting position —robot parkour being an example [1311, 464]. Achieving this navigation behavior does require access to an odometry estimate, since the robot needs to keep track of the progress towards the goal in an inertial frame. This suggests that state estimation is still needed to achieve more complex locomotion and navigation tasks.

A few works have proposed to learn state estimation as part of the locomotion policy learning process [516], and explicitly estimate variables such as the body velocity, feet height, and contact state. While this has only been used for locomotion purposes, it can be a promising alternative to obtain more accurate leg odometry estimates for proprioceptive state estimation or odometry factors in SLAM.

Humanoid Robots. Humanoid robots embed part of the dreams that have motivated the development of robotics —creating artificial agents able to do the dull, dangerous, and dirty tasks that humans prefer not to do. Having a human-like body should —in principle— enable them to seamlessly work in human-oriented environments, using tools, devices, and even vehicles designed for people’s use.

The DARPA Robotics Challenge, briefly introduced in Section 12.1.1, has been one of the main eforts in this direction. The diverse set of tasks, involving locomotion on rough terrain but also tool handling and driving vehicles, presented several challenges towards this goal. However, after it ended in 2015, most of the eforts in legged robotics focused on quadrupedal platforms instead —which presented clear advantages in control and robustness, motivating their adoption for industrial inspection and monitoring. It was not until 2021 when a commercial interest in humanoid platforms arose again, motivated by the optimism and fast-pacing progress in artificial intelligence (AI), as well as the success of quadrupedal robots.

Diverse companies have recently aimed to develop new humanoid platforms as a way to embody AI systems in the real world. Humanoid robots have been targeted to solve complex tasks in delivery, warehousing, and manufacturing —working sideby-side with people, in highly demanding environments. This presents diferent challenges that push the topics covered in this chapter in directions currently unexplored. Problems such as long-term, accurate and reliable whole-body estimation need to be solved for humanoid robots to be able to achieve tasks in last-mile delivery problems. Intermittent contacts, from the feet but also the trunk, arms, and hands are expected when handling parcels or other objects in a warehouse setting. Similarly, compliance is required when operating close to people in order to be safe —this also requires relaxing the rigid contact assumptions we made in this chapter.

## Acknowledgments

The authors thank Michele Focchi (University of Trento) for his advice in preparing parts of this chapter.

PART III FROM SLAM TO SPATIAL AI

III

Prelude

Marc Pollefeys Ayoung Kim, Frank Dellaert, Timothy Barfoot, Luca Carlone, and Daniel Cremers

In Part I, we explored the foundational principles of SLAM, followed by an indepth look at SLAM systems across various sensor modalities in Part II. Building on this foundation, Part III extends SLAM into the broader domain of Spatial AI. We begin this part with a perennial question that has persistently challenged the SLAM community.

## Is SLAM a solved problem?

The answer could be yes —if we limit ourselves to small, static, indoor environments rich in features. Yet such controlled settings are far from where SLAM’s true promise lies. We imagine SLAM as the guiding light for robots venturing into realms beyond human reach —dark, hostile, and uncharted. We dream of underwater explorers diving to depths no human could ever fathom, mapping the mysteries of the abyss. We want SLAM to navigate robotic explorers on Mars, analyzing alien terrains and environments. We yearn for SLAM to lead rescue missions in disasterstricken zones, locating survivors and saving lives. And, of course, we want SLAM to be deployed in highly dynamic, human-centric, and city-scale environments, say in a self-driving car in dense down-town trafic.

Parts I and II covered the fundamental concepts necessary to reach this final goal, which is intertwined with many real-world challenges. One thing is certain: we need greater intelligence in perceiving and understanding the world, making the transition to spatial AI essential.

## III.1 Spatial Artificial Intelligence

What is the key diference between conventional SLAM and spatial AI? The first thing to note is the integration of recent advances in deep learning for spatial understanding. Transitioning to learning-based frameworks enhances the ability to handle complex, dificult-to-model scenarios, resulting in more adaptive and robust performance. Beyond performance improvements, spatial AI will transform and expand space representation by enabling novel view synthesis, enriching maps with semantic information, and securing flexibility from dynamic scene changes.

The emerging role of foundation models in computer vision and robotics must be harnessed for advancing spatial AI. At the same time, it is crucial not to overlook the importance of real-world deployment, particularly by addressing the computational aspects of the system components.

## III.2 Spatial AI Applications

Spatial AI promises to enable devices to understand the environments they navigate in, to assist people or enable (physical) autonomous agents to carry out tasks. While multi-modal foundation models trained on images and accompanying text can provide a general explanation of observed scenes, many tasks require to build up richer persistent scene representations. In fact, many image foundation models struggle altogether with spatial understanding in images as they only provide a holistic scene description. By combining SLAM and semantic image understanding capabilities, persistent 3D semantic scene representations can be build and kept up to date, creating spatio-temporal memories of environments. Spatial AI, particularly when combined with SLAM, is transforming various domains by enabling devices to understand and interact with their physical environment in real time. Even more can be achieved when building a shared spatial representation through crowd mapping. This section describes the application of Spatial AI to AR/VR glasses, humanoid robots, self-driving vehicles, drones, industrial automation, emergency response and healthcare.

## III.2.1 AR/VR Glasses

In augmented and virtual reality, Spatial AI with SLAM enables immersive, contextaware experiences by allowing AR/VR glasses to understand and interact with the physical world in real-time. SLAM helps these devices map their surroundings and track their position within that space, which is essential for anchoring virtual objects accurately in the user’s environment. For example, in AR applications, this allows digital content to remain fixed to real-world surfaces as the user moves, enhancing realism and usability. In VR, it supports inside-out tracking, eliminating the need for external sensors and enabling more natural movement and interaction.

Moreover, Spatial AI can interpret semantic information from the environment —like identifying walls, furniture, or people— which allows for more intelligent and adaptive applications. This is particularly useful in collaborative AR scenarios, where multiple users interact with the same virtual content in a shared space, or in accessibility tools that provide spatial cues to visually impaired users.

## III.2.2 AI Glasses

AI glasses represent a specialized evolution of AR wearables, integrating Spatial AI to deliver real-time contextual intelligence directly to the user. Unlike generalpurpose AR/VR headsets, AI glasses are designed for lightweight, continuous use in everyday environments. With SLAM, these glasses can localize themselves and map the surrounding space, enabling persistent spatial awareness and interaction with digital content anchored to the real world.

These capabilities unlock a range of applications —from real-time translation and navigation assistance to object recognition and task guidance. For instance, in industrial settings, AI glasses can provide step-by-step instructions for machinery, while in retail, they can provide product information or inventory data as the user looks around. When paired with voice interfaces and edge AI processing, these glasses become powerful tools for hands-free computing, enhancing productivity, accessibility, and situational awareness in both professional and consumer contexts.

## III.2.3 Humanoid Robots

For humanoid robots, Spatial AI is foundational to achieving autonomous, humanlike navigation and interaction. These robots must perceive and understand complex, dynamic environments —such as homes, ofices, or hospitals— and SLAM enables them to build and update maps of these spaces while simultaneously localizing themselves within them. This is critical for tasks like fetching objects, guiding people, or performing inspections.

Beyond navigation, Spatial AI allows humanoid robots to reason about their surroundings. For instance, they can recognize and avoid obstacles, understand spatial relationships (e.g., “the cup is on the table”), and plan actions accordingly. When combined with vision and language models, this spatial understanding enables more intuitive human-robot interaction, where robots can follow natural language commands like “go to the kitchen and bring me the red mug.”

## III.2.4 Self-Driving Vehicles

In autonomous vehicles, Spatial AI and SLAM are key to safe and eficient navigation in both structured and unstructured environments. SLAM allows vehicles to create high-fidelity maps of their surroundings in real time, which is especially valuable in areas where GPS is unreliable or unavailable, such as tunnels or dense urban canyons. These maps help the vehicle localize itself precisely and detect changes in the environment, such as construction zones or temporary obstacles.

Spatial AI enhances this by adding semantic understanding —identifying lanes, trafic signs, pedestrians, and other vehicles. This enables more sophisticated decisionmaking, such as predicting pedestrian intent or negotiating complex intersections.

In combination with other sensors like LiDAR and radar, SLAM-based Spatial AI forms the backbone of perception systems that allow self-driving cars to operate safely and autonomously in diverse conditions.

## III.2.5 Drones and Aerial Robotics

In aerial robotics, Spatial AI with SLAM enables drones to autonomously navigate complex environments without relying on GPS. This is especially valuable in indoor, underground, or densely forested areas where GPS signals are weak or unavailable. SLAM allows drones to build 3D maps of their surroundings in real time, avoid obstacles, and plan eficient flight paths. Combined with semantic understanding, drones can identify objects of interest —such as power lines, crops, or structural damage— making them invaluable for inspection, agriculture, and disaster response.

## III.2.6 Industrial Automation and Warehousing

In smart factories and warehouses, Spatial AI empowers mobile robots and automated guided vehicles (AGVs) to operate safely and eficiently alongside human workers. SLAM enables these systems to localize themselves and dynamically update their maps as the environment changes —such as when inventory is moved or new obstacles appear. Spatial AI also supports task-level reasoning, allowing robots to understand spatial relationships (e.g., shelf locations, aisle widths) and optimize logistics operations like picking, packing, and delivery.

## III.2.7 Emergency Response

In emergency response scenarios, Spatial AI and SLAM are invaluable for search and rescue operations, disaster assessment, and recovery eforts. Drones and ground robots equipped with SLAM can navigate hazardous environments, such as collapsed buildings or flood zones, to locate survivors and assess damage. These systems can create real-time maps of the afected areas, providing critical information to first responders and enabling more eficient and safer operations, in particular if those have access to AR/VR glasses or other wearables.

Spatial AI also supports the deployment of autonomous vehicles and robots to deliver supplies, medical aid, and communication equipment to areas that are difficult to access. By understanding and mapping the environment, these systems can navigate complex terrains and avoid obstacles, ensuring timely and accurate delivery of essential resources.

## III.2.8 Healthcare and Assistive Technologies

Spatial AI is increasingly used in assistive devices for people with disabilities. For example, wearable devices equipped with SLAM can help visually impaired users navigate unfamiliar environments by providing real-time spatial cues and obstacle warnings. Semantic understanding of the environment is critical to provide efective assistance. In hospitals, autonomous service robots use Spatial AI to deliver medications, transport supplies, and guide patients, reducing staf workload and improving operational eficiency.

## III.3 How to Read Part III?

The organization of this part follows a topic-wise chapter categorization of how SLAM is advancing toward Spatial AI, leveraging recent learning-based approaches.

In Chapter 13, we introduce how deep learning techniques can enhance and extend conventional SLAM systems. Recent progress in novel view synthesis and its integration into SLAM frameworks is discussed in Chapter 14. Approaches for handling dynamic environments and structural changes are presented in Chapter 15. In Chapter 16, we explore the transition from purely metric maps to semantically enriched representations for higher-level scene understanding. The integration of large language models to support spatial reasoning is examined in Chapter 17. Finally, we investigate the computational structures and system-level design considerations essential for enabling Spatial AI in Chapter 18.

## 13

