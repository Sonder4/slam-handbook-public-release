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
