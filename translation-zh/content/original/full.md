# SLAM Handbook From Localization and Mapping to Spatial Intelligence

Compiled on

April 21, 2026

Edited by

Luca Carlone, Ayoung Kim, Timothy Barfoot, Daniel Cremers, and Frank Dellaert

## IMPORTANT NOTE ON RELEASE:

© Cambridge University Press. No reproduction of any part may take place without the written permission of Cambridge University Press.

## Authored by

Henrik Andreasson Timothy Barfoot Martin B¨uchner Luca Carlone Margarita Chli Daniel Cremers Jia Deng Jakob Engel C´edric Le Gentil Connor Holmes Nathan Hughes Kasra Khosoussi John Leonard Martin Magnusson Mat´ıas Mattamala Mustafa Mukadam Helen Oleynikova Marc Pollefeys David Rosen Jingnan Shi Juan D. Tard´os Teresa Vidal-Calleja Heng Yang Shibo Zhao

Arash Asgharivaskasi Jens Behley Cesar Cadena Yun Chang Henrik Christensen Andrew J. Davison Gamini Dissanayake Maurice Fallon Christofer Heckman Guoquan Huang Krishna Murthy Jatavallabhul Ayoung Kim Stefan Leutenegger Joshua Mangelson Jos´e M Mart´ınez Montiel Jose Neira Lionel Ott Victor Reijgwart Davide Scaramuzza Cyrill Stachniss Zachary Teed Chen Wang Fu Zhang

Nikolay Atanasov Jose Luis Blanco-Claraco Marco Camurri Boris Chidlovskii Javier Civera Frank Dellaert Kevin Doherty Guillermo Gallego Javier Hidalgo-Carri´o Shoudong Huang Michael Kaess Giseop Kim Dominic Maggio Hidenobu Matsuk Sacha Morin Paul Newman Liam Paull Jerome Revaud Lukas Schmid Niko Sunderhauf Abhinav Valada Felix Wimbauer Ji Zhang

## Contents

List of Contributors page viii
Foreword x
Preface xii
Notation xiv

PART I FOUNDATIONS OF SLAM 1  
I Prelude 3  
I.1 What is SLAM? 3  
I.2 Anatomy of a Modern SLAM System 5  
I.3 The Role of SLAM in the Autonomy Architecture 9  
I.4 Past, Present, and Future of SLAM, and Scope of this Handbook 13  
I.5 Handbook Structure 17  
1 Factor Graphs for SLAM 19  
1.1 Visualizing SLAM With Factor Graphs 20  
1.2 From MAP Inference to Least Squares 25  
1.3 Solving Linear Least Squares 29  
1.4 Nonlinear Optimization 33  
1.5 Factor Graphs and Sparsity 35  
1.6 Elimination 40  
1.7 Incremental SLAM 46  
1.8 Further Readings & Recent Trends 51  
2 Advanced State Variable Representations 53  
2.1 Optimization on Manifolds 53  
2.2 Continuous-Time Trajectories 62  
2.3 Further Readings & Recent Trends 74  
3 Robustness to Incorrect Data Association and Outliers 75  
3.1 What Causes Outliers and Why Are They a Problem? 75  
3.2 Detecting and Rejecting Outliers in the SLAM Front-end 78

iv Contents
3.3 Increasing Robustness to Outliers in the SLAM Back-end 85
3.4 Further Readings & Recent Trends 98
4 Differentiable Optimization 101
4.1 Recap on Nonlinear Least Squares 102
4.2 Differentiation Through Nonlinear Least Squares 103
4.3 Differentiation on Manifold 109
4.4 Numerical Challenges of Automatic Differentiation and Modern Libraries 113
4.5 Further Readings & Recent Trends 118
5 Dense Map Representations 119
5.1 Range Sensing Preliminaries 119
5.2 Foundations of Dense Mapping 122
5.3 Map Representations 126
5.4 Constructing Maps: Methods and Practices 135
5.5 Usage Considerations 145
6 Certifiably Optimal Solvers and Theoretical Properties of SLAM 14
6.1 Certifiably Optimal Solvers for SLAM 150
6.2 How Accurate is the Optimal Solution of a SLAM Problem? 173
6.3 Further Readings & Recent Trends 178
PART II SLAM IN PRACTICE 183
II Prelude 185
II.1 Key Modules in the SLAM Front-End 185
II.2 Sensors and Factor Graphs 187
II.3 Evaluation 191
II.4 How to Read Part II? 192
7 Visual SLAM 193
7.1 Historical Background and Terminology 193
7.2 The Processing Pipeline of a Visual SLAM System 196
7.3 Visual SLAM Fundamentals 197
7.4 Further Considerations about Image Alignment and BA 207
7.5 Examples of Full Visual SLAM Systems 215
7.6 Real-time Dense Reconstruction 216
7.7 SLAM with Depth-sensing Cameras 217
7.8 Combining Vision with Other Modalities 219
7.9 Further Readings & Recent Trends 222
8 LiDAR SLAM 224
8.1 LiDAR Sensing Preliminaries and Categorization 225
8.2 LiDAR Odometry 227

Contents v
8.3 LiDAR Place Recognition 236
8.4 LiDAR SLAM 240
8.5 Further Readings & Recent Trends 248
9 Radar SLAM 250
9.1 Introduction to Radar 250
9.2 Radar Odometry 261
9.3 Radar Place Recognition 269
9.4 Radar SLAM 273
9.5 Radar Datasets 279
9.6 Further Readings & Recent Trends 280
10 Event-based SLAM 282
10.1 Sensor Description 282
10.2 Challenges and Applications 286
10.3 Overview and Taxonomy of Event-based SLAM Methods 287
10.4 Front-end of an Event-based SLAM System 289
10.5 Back-end of an Event-based SLAM System 293
10.6 State-of-the-Art Systems 294
10.7 Datasets, Simulators, and Benchmarks 294
10.8 Further Readings & Recent Trends 302
11 Inertial Odometry for SLAM 304
11.1 Basics of Inertial Sensing and Navigation 304
11.2 IMU Preintegration and Factor Graphs 308
11.3 Observability of Aided Inertial Navigation 319
11.4 Visual-Inertial Odometry and Practical Considerations 326
11.5 Further Readings & Recent Trends 330
12 Leg Odometry for SLAM 333
12.1 Historical Background and Preliminaries 333
12.2 Motion Estimation 342
12.3 Contact Estimation 345
12.4 Using Leg Odometry for State Estimation 348
12.5 Open Challenges 353
12.6 Further Readings & Recent Trends 355
PART III FROM SLAM TO SPATIAL AI 359
III Prelude 361
III.1 Spatial Artificial Intelligence 361
III.2 Spatial AI Applications 362
III.3 How to Read Part III? 365

## Contents

13 Boosting SLAM with Deep Learning 366
13.1 Deep Learning for Depth and Camera Pose 368
13.2 Deep Learning for Feature Matching and Optical Flow 374
13.3 Differentiable Bundle Adjustment and DROID-SLAM 379
13.4 DuSt3R 386
13.5 MASt3R 392
13.6 Extending MASt3R to structure from motion (SFM) and SLAM 393
13.7 Further Readings & Recent Trends 395

14 Map Representations with Differentiable Volume Rendering 397
14.1 3D Scene Representation and Differentiable Rendering 398
14.2 Neural Radiance Fields (NeRF) 401
14.3 3D Gaussian Splatting 409
14.4 Further Readings & Recent Trends 414

15 Dynamic and Deformable SLAM 417
15.1 Characterizing the Dynamic SLAM Problem 418
15.2 Short-term Dynamics and Dynamic SLAM 423
15.3 Long-term Dynamic and Lifelong SLAM 432
15.4 Deformable SLAM 446
15.5 Further Readings & Recent Trends 451

16 Metric-Semantic SLAM 454
16.1 From Traditional SLAM to Metric-Semantic SLAM 455
16.2 Sparse Metric-Semantic Representations 456
16.3 Dense Metric-Semantic Representations 471
16.4 Hierarchical Metric-Semantic Representations and 3D Scene Graphs 482
16.5 Further Readings & Recent Trends 487

17 Towards Open-World Spatial AI 490
17.1 Background and Terminology 491
17.2 Foundation Models for Spatial AI 495
17.3 Open-World Mapping 499
17.4 Further Readings & Recent Trends 514

18 The Computational Structure of Spatial AI Systems 521
18.1 From SLAM to Spatial AI 521
18.2 Overall Computational Structure 525
18.3 State Estimation and Machine Learning in Spatial AI 526
18.4 The Future Landscape of Processor and Sensor Hardware 528
18.5 Mapping Spatial AI Graphs to Hardware 532
18.6 Convergent Distributed Computation with Gaussian Belief Propagation 540
18.7 Continual Learning within Factor Graphs 542

Contents vii
18.8 Performance Metrics 545
18.9 Further Readings & Recent Trends 547
Epilogue 548
References 551
Author index 645
Subject index 647

## List of Contributors

Henrik Andreasson, Orebro University<sup>¨</sup>

Arash Asgharivaskasi, University of California San Diego

Nikolay Atanasov, University of California San Diego

Timothy D. Barfoot, University of Toronto

Jens Behley, University of Bonn

Jose Luis Blanco-Claraco, University of Almer´ıa

Martin B¨uchner, University of Freiburg

Cesar Cadena, ETH Zurich

Marco Camurri, University of Trento

Luca Carlone, Massachusetts Institute of Technology

Yun Chang, Massachusetts Institute of Technology

Boris Chidlovskii, NAVER LABS

Margarita Chli, ETH Zurich & University of Cyprus

Henrik Christensen, University of California San Diego

Javier Civera, University of Zaragoza

Daniel Cremers, Technical University of Munich

Andrew J. Davison, Imperial College London

Frank Dellaert, Georgia Tech

Jia Deng, Princeton University

Gamini Dissanayake, University of Technology Sydney

Kevin Doherty, Boston Dynamics

Jakob Engel, Meta Reality Labs

Maurice Fallon, University of Oxford

Guillermo Gallego, Technical University of Berlin

C´edric Le Gentil, University of Technology Sydney

Christofer Heckman, University of Colorado Boulder

Javier Hidalgo-Carri´o, University of Zurich

Connor Holmes, University of Toronto

Guoquan Huang, University of Delaware

Shoudong Huang, University of Technology Sydney

Nathan Hughes, Massachusetts Institute of Technology

Krishna M. Jatavallabhula, Massachusetts Institute of Technology

Michael Kaess, Carnegie Mellon University

Kasra Khosoussi, University of Queensland

Ayoung Kim, Seoul National University

Giseop Kim, NAVER LABS

John Leonard, Massachusetts Institute of Technology

Stefan Leutenegger, ETH Zurich

Dominic Maggio, Massachusetts Institute of Technology

Martin Magnusson, Orebro University<sup>¨</sup>

Joshua Mangelson, Brigham Young University

Hidenobu Matsuki, Imperial College London

Mat´ıas Mattamala, University of Oxford

Jos´e M. Martinez Montiel, University of Zaragoza

Sacha Morin, University of Montreal

Mustafa Mukadam, Amazon

Jose Neira, University of Zaragoza

Paul Newman, University of Oxford

Helen Oleynikova, ETH Zurich

Lionel Ott, ETH Zurich

Liam Paull, University of Montreal

Marc Pollefeys, ETH Zurich

Victor Reijgwart, ETH Zurich

Jerome Revaud, NAVER LABS

David Rosen, Northeastern University

Davide Scaramuzza, ETH Zurich

Lukas Schmid, Massachusetts Institute of Technology

Jingnan Shi, Massachusetts Institute of Technology

Cyrill Stachniss, University of Bonn

Niko Sunderhauf, Queensland University of Technology

Juan D. Tard´os, University of Zaragoza

Zachary Teed, Princeton University

Abhinav Valada, University of Freiburg

Teresa Vidal-Calleja, University of Technology Sydney

Chen Wang, University at Bufalo

Felix Wimbauer, Technical University of Munich

Heng Yang, Harvard University

Fu Zhang, The University of Hong Kong

Ji Zhang, Carnegie Mellon University

Shibo Zhao, Carnegie Mellon University

# Foreword

Simultaneous Localization and Mapping —better known as SLAM— refers to the fundamental problem of building spatial models of an environment while simultaneously determining the position of a robot within that environment. The term itself was first coined in 1995 by Hugh Durrant-Whyte and John Leonard, marking the formalization of a problem that sits at the intersection of robotics, geometry, controls, and probabilistic inference.

SLAM is as elegant as it is formidable. At its core, it addresses the challenge of reasoning over high-dimensional, uncertain, and dynamic systems. The process demands precise spatial inference and robust probabilistic modeling to build coherent maps of the world —maps that must be constructed in real time, often under conditions of noise and ambiguity.

What makes SLAM particularly compelling is its universality. In computer vision, it is mirrored in the problem of Structure from Motion; in robotics, it underpins everything from indoor autonomous navigation to planetary exploration and selfdriving cars. Since its inception, SLAM has inspired tens of thousands of research papers, drawing deeply from disciplines as diverse as physics, statistics, computer vision, geometry, controls, and machine learning. Its evolution has catalyzed the development of increasingly capable autonomous systems, able to operate at scale in complex, open-world environments.

This volume brings together contributions from some of the field’s foremost experts and rising stars. The chapters represent the state of the art in SLAM today, reflecting both the depth of theoretical innovations and the breadth of practical applications. From its early formulations based on Kalman filters and Bayesian estimation, SLAM has matured into a rich tapestry of mathematical frameworks

encompassing graph-based optimization, factor graphs, nonlinear least squares, and deep learning-based techniques. Beyond introducing the mathematical foundations of SLAM, this volume provides valuable guidance to the practitioner by discussing real-world use cases ranging from vision-based and LiDAR-based SLAM systems to legged locomotion. It also covers recent developments in Spatial AI, showing how advances in deep learning, diferentiable rendering, and large vision

## Foreword

and language models point the way toward representations that provide robots with a rich spatial and semantic understanding of their environment. The real-world impact of SLAM is unmistakable. Whether embedded in robotic vacuum cleaners or powering fleets of autonomous vehicles, SLAM is now a cornerstone technology that enables intelligent systems to perceive and act in the physical world. It represents one of the most sophisticated and enduring eforts in robotics to date, and continues to shape how machines understand space, uncertainty, and motion. The SLAM research community is one of the most vibrant and technically rigorous within robotics. Many of today’s leading roboticists began their careers solving SLAM, and have since extended its principles into domains such as state estimation, control, sensor fusion. SLAM stands as one of the defining problems of probabilistic robotics —a field that, through SLAM, has helped usher statistical reasoning into the heart of robotic perception and autonomy. As contributors to and beneficiaries of this remarkable community, we are proud to introduce this handbook. It ofers a comprehensive view of SLAM at its current frontier —building upon three decades of foundational work while pointing the way to the challenges and opportunities ahead. The authors represented here are the next generation of leaders, and their work sets the tone for what is possible when rigorous science meets real-world complexity. We celebrate their contributions, and we believe this book will serve as both a vital reference and an inspiration for researchers, engineers, and students in the years to come.

Wolfram Burgard, Dieter Fox, and Sebastian Thrun

## Preface

We are proud to present this handbook on SLAM after over two years of efort. Our vision was to produce a primer that could be handed to upper-year undergraduates or new graduate students that summarizes the current state of the art. We have attempted to produce a unified document where chapters build on one another. This was quite challenging with 70 authors simultaneously writing but we are delighted with the result: a book by the community, for the community.

The term SLAM itself is about 30 years old, but the idea of building a map while localizing on it goes back much further. In places throughout the handbook we look at this historical path, but we are mainly focused on the present and the future of SLAM here. Accordingly, the handbook is divided into three parts:

Part I: We introduce the foundations of SLAM, focusing on the estimation-theoretic machinery of the back-end and the map representations it produces. We cover factor graphs, manifold state representation, handling outliers, diferentiable optimization, dense mapping, and advanced solvers.

Part II: Here we study SLAM in practice and focus on the characteristics and integration of various sensor modalities used in SLAM, including RGB cameras, LiDARs, event cameras, radars, IMUs, and robot kinematics. We explain how these sensors contribute to odometry, loop closure, and other factors, and how they can be calibrated, synchronized, fused, and evaluated within a factor-graph framework.

Part III: We look forward to emerging trends in SLAM and Spatial AI, exploring deep learning integration, novel map representations, and operation in dynamic or deformable environments. We also consider semantic reasoning, language grounding, and future computational architectures for distributed spatial perception.

We would like to thank all the authors for their invaluable contributions. They are all experts in their respective disciplines and working with them has been an honor and a pleasure. We would also very much like to thank Wolfram Burgard, Dieter Fox, and Sebastian Thrun, the authors of the famous Probabilistic Robotics book, for writing the foreword to this handbook; we are certainly standing on the shoulders of giants. We thank Oussama Khatib, co-editor of the Handbook of Robotics, for his

## Preface

seasoned advice on writing this handbook. We are especially grateful to Frank Park for his leadership in proposing a SLAM handbook two years ago; he was there from the start. We are indebted to Dongjae Lee, who helped immensely with the nuts and bolts of producing such a large manuscript, and to Vendi Pavic for her invaluable assistance in preparing the final materials and coordinating the submission process for this handbook. We also thank the IEEE RAS Technical Committee on Computer & Robot Vision for advertising and boosting the impact of this project. Finally, we are grateful to Navid Mahmoudian Bidgoli, Andrew Kramer, Chris Ofner, and Adam Pooley for providing comments on an early version of this handbook, and to Cambridge University Press for supporting this ambitious and hopefully impactful project.

It has been a wonderful journey bringing this handbook to fruition. We hope you enjoy reading these pages as much as we enjoyed bringing them together.

Luca Carlone, Ayoung Kim, Timothy Barfoot, Daniel Cremers, and Frank Dellaert

## Notation

– General Notation –

a This font is used for real scalars

a This font is used for real column vectors

A This font is used for real matrices

A This font is used for sets

I The identity matrix

0 The zero matrix

$A ^ { \mathsf { T } }$ The transpose of matrix A

$\mathbb { R } ^ { M \times N }$ The vector space of real $M \times N$ matrices

$p ( a )$ The probability density of a

$p ( { \pmb a } | { \pmb b } )$ The probability density of a given b

$p ( \boldsymbol a ; \boldsymbol b )$ The probability density of a parametrized by b

$\scriptstyle { \mathcal { N } } ( \mu , \Sigma )$ Gaussian probability density with mean $\pmb { \mu }$ and covariance Σ

$\mathcal { G P } ( \pmb { \mu } ( t ) , \pmb { \kappa } ( t , t ^ { \prime } ) )$ Gaussian process with mean function, $\pmb { \mu } ( t )$ , and covariance function, $\kappa ( t , t ^ { \prime } )$

<sup>ˆ</sup>( ) A posterior (estimated) quantity

<sup>ˇ(</sup>·<sup>)</sup> <sup>A</sup> <sup>prior</sup> <sup>quantity</sup>

( ) The value of a quantity at timestep k

$( \cdot ) _ { k _ { 1 } : k _ { 2 } }$ The set of values of a quantity from timestep $k _ { 1 }$ to timestep $k _ { 2 } .$ , inclusive

$\left\| \cdot \right\| _ { 1 }$ L1 norm $\| \pmb { x } \| _ { 1 } = \sum$ |<sup>x</sup>i|

$\lVert \cdot \rVert _ { 2 }$ L2 norm $\| \pmb { x } \| _ { 2 } = \sqrt { \sum x _ { i } ^ { 2 } }$

## Notation

– 3D Geometry Notation –

${ \mathcal { F } } ^ { a }$ A reference frame in three dimensions

${ \pmb v } ^ { a }$ The coordinates of a vector in frame ${ \mathcal { F } } ^ { a }$

$R _ { a } ^ { b }$ A $3 \times 3$ rotation matrix (member of $\mathrm { S O ( 3 ) } )$ that takes points expressed in ${ \mathcal { F } } ^ { a }$ and re-expresses them in (purely rotated) $\mathcal { F } ^ { b } \colon v ^ { b } = R _ { a } ^ { b } v ^ { a }$

$\pmb { t } _ { a } ^ { b }$ The three-dimensional position of the origin of frame ${ \mathcal { F } } ^ { a }$ expressed in $\mathcal { F } ^ { b }$

$\tilde { v } ^ { a } = \left[ { \begin{array} { l } { v ^ { a } } \\ { 1 } \end{array} } \right]$ $\mathrm { ~ A ~ 4 ~ } \times \mathrm { ~ 1 ~ }$ homogeneous point expressed in ${ \mathcal { F } } ^ { a }$

$\pmb { T } _ { a } ^ { b } = \left[ \begin{array} { l l } { \pmb { R } _ { a } ^ { b } } & { \pmb { t } _ { a } ^ { b } } \\ { \pmb { 0 } } & { 1 } \end{array} \right]$ A $4 \times 4$ transformation matrix (member of $\mathrm { S E ( 3 ) } )$ that takes homogeneous points expressed in ${ \mathcal { F } } ^ { a }$ and re-expresses them in (rotated and translated) $\mathcal { F } ^ { b } \colon \tilde { { \boldsymbol { v } } } ^ { b } = T _ { a } ^ { b } \tilde { { \boldsymbol { v } } } ^ { a }$

SO(3) The special orthogonal group, a matrix Lie group used to represent 3D rotations

so(3) The Lie algebra associated with SO(3)

SE(3) The special Euclidean group, a matrix Lie group used to represent 3D poses

se(3) The Lie algebra associated with $\operatorname { S E } ( 3 )$

( )∧ An operator mapping a vector in $\mathbb { R } ^ { 3 }$ (resp. $\mathbb { R } ^ { 6 } )$ to an element of the Lie algebra for 3D rotations (resp. 3D poses); implements the cross product for three-dimensional quantities, $i . e .$ , for two vectors $\mathbf { \boldsymbol { \mathsf { u } } } , \mathbf { \boldsymbol { \mathsf { v } } } \in \mathbb { R } ^ { 3 } , \mathbf { \boldsymbol { \mathsf { u } } } ^ { \wedge } \mathbf { \boldsymbol { \mathsf { v } } } = \mathbf { \boldsymbol { \mathsf { u } } } \times \mathbf { \boldsymbol { \mathsf { v } } }$

( )∨ An operator mapping an element of the Lie algebra for 3D rotations (resp. 3D poses) to a vector in $\mathbb { R } ^ { 3 } \ ( \mathrm { r e s p . } \ \mathbb { R } ^ { 6 } )$

PART I

FOUNDATIONS OF SLAM

Prelude

Luca Carlone, Ayoung Kim, Frank Dellaert, Timothy Barfoot, and Daniel Cremers

This chapter introduces the Simultaneous Localization and Mapping (SLAM) problem, presents the modules that form a typical SLAM system, and explains the role of SLAM in the architecture of an autonomous system. The chapter also provides a short historical perspective of the topic and discusses how the traditional notion of SLAM is evolving to fully leverage new technological trends and opportunities. The ultimate goal of the chapter is to introduce basic terminology and motivations, and to describe the scope and structure of this handbook.

## I.1 What is SLAM?

A necessary prerequisite for a robot to operate safely and efectively in an unknown environment is to form an internal representation of its surroundings. This representation can be used to support obstacle avoidance, low-level control, planning, and, more generally, the decision-making processes required for the robot to complete the task it has been assigned. The execution of simple tasks (e.g., following a lane, or maintaining a certain distance to an object in front of the robot) may only require tracking entities of interest in the sensor data streams, while complex tasks (e.g., large-scale navigation or mobile manipulation) require building and maintaining a persistent representation (a map) of the environment. Such a map describes the presence of obstacles, objects, and other entities of interest, and their relative location with respect to the robot’s pose (position and orientation). For instance, the map might be used to instruct the robot to reach a location of interest, to grasp a certain object, or to support the exploration of an initially unknown environment.

For a robot operating in an initially unknown environment, the problem of building a map of the environment, while concurrently estimating its pose with respect to that map, is referred to as simultaneous localization and mapping (SLAM). Figure I.1 provides some real-world examples of SLAM in action. SLAM reduces to localization if the map is given, in which case the robot only has to estimate its pose with respect to the map. On the other hand, SLAM reduces to mapping if the pose of the robot is already known, for instance when an absolute positioning system is

![](images/0227294e4d9afbabbfba58a29e53d164899da8c48edf39f51e834cc86ea2066f.jpg)  
Warehouse Robot Using Visual SLAM to Navigate (credit: Canon Inc.)

![](images/cfa753e057d87f976c6440a846d8f742c7a41a9fd8f84ed94f893d425e451db5.jpg)  
Lidar SLAM Used in the Forestry Industry to Inventory Trees (credit: Kukko et al. (2017))

![](images/39e5eb07feefbf895ae2c1b4ae6ae6808deb358ea6833cd02e4f5879d810b8ac.jpg)  
Floor-Cleaning Robot Using SLAM to Navigate (credit: Robot Vacuum Company)

![](images/b22277e3610ad0e86e9a839b9e9b53eaea1b2810a679479e754a432d42619530.jpg)  
Self-Driving Car Navigating with High-Definition Map Created by SLAM (credit: Cruise)

![](images/6b2d06589a93b18b2ec8c1cb756b3a549d50680ab30c758f5eb06950a918cd46.jpg)  
Drone Mapping As-Built Construction via Visual SLAM (credit: Skydio)

![](images/213386f35e5da17e7e4aba003f387ee81b12d57815168c28719289608f6859b2.jpg)  
Legged Robot Using SLAM to Map a Building (credit: Boston Dynamics)

![](images/8c48ecd57d46879bbdb5db6d2594881889ad8bbdcc08dc549e39e7459a244dd3.jpg)  
Wearable Devices Using SI AM to Build Augmented Reality Scenes (credit: Meta Reality Labs)

Figure I.1 SLAM is rapidly becoming an enabling technology in a wide array of applications including warehouse robotics, forest inventories [613], floor-cleaning, self-driving cars, drones surveillance, legged-robot mapping, and augmented reality to name only a few. (©2017 Elsevier)

used (e.g., diferential GPS or motion capture), in which case the robot only needs to model its surroundings using its sensor data.

The central role of SLAM in robotics research is due to the fact that robot poses are rarely known in practical applications. Diferential GPS and motion capture systems are expensive and restricted to small areas, hence being unsuitable for largescale robot deployments. Consumer-grade GPS is much more broadly available, but its accuracy (with errors typically in the order of meters) and its availability (which is limited to outdoor areas with line-of-sight to satellites) often makes it unsuitable as a source of localization; the consumer-grade GPS —when available— is typically used as an additional source of information for SLAM, rather than a replacement for the localization aspects of SLAM.

Similarly, in many robotics applications, the robot will not typically have access to a prior map, hence it needs to perform SLAM rather than localization. Indeed, in certain applications, building a map is actually the goal of the robot deployment; for instance, when robots are used to support disaster response and search-andrescue operations, they might be deployed to construct a map of the disaster site to help first-responders. In other cases, the map might be stale or not have enough detail. For instance, a domestic robot might have access to the floor plan of the apartment it has to operate in, but such a floor plan may not describe the furniture and objects actually present in the environment, nor the fact that these elements can be rearranged from day to day. In a similar manner, Mars exploration rovers have access to low-resolution satellite maps of the Martian surface, but they still need to perform local mapping to guide obstacle avoidance and motion planning.

The importance of the SLAM problem motivates the large amount of attention this topic has received, both within the research community and from practitioners interested in using SLAM technologies across multiple application domains from robotics, to virtual and augmented reality. At the same time, SLAM remains an exciting area of research, with many open problems and new opportunities.

## I.2 Anatomy of a Modern SLAM System

The ultimate goal of SLAM is to infer a map representation and robot poses (i.e., the robot trajectory) from sensor data, including data from proprioceptive sensors (e.g., wheel odometry or inertial measurement unit, IMU) and exteroceptive sensors (e.g., cameras, light detection and ranging (LiDAR), radars). In mathematical terms this can be understood as an inverse problem: given a set of measurements, the goal is to determine a model of the world (the map) and a set of robot poses (trajectory) that could have produced those measurements. There exist two alternative strategies to solve the SLAM problem: indirect and direct methods.

The vast majority of SLAM methods prefers pre-processing the raw sensory data in order to extract “intermediate representations” that are compact and easier to describe mathematically. For instance, in visual SLAM, instead of using every pixel in a camera image, these indirect methods extract a few distinctive 2D point features (or keypoints) and then only model the geometry of how these keypoints depend on the pose of the camera and the geometry of the scene. In contrast, rather than computing an intermediate abstraction, direct methods aim to compute localization and mapping directly from the raw sensory data. This categorization is prominent in visual SLAM but is not limited to it as we will see in Chapter 8 and Chapter 9. Both indirect and direct methods have their advantages and shortcomings.

Indirect methods are often faster and more memory eficient, since they merely process a small subset of keypoints for which the 3D location is determined. As a consequence, real-time capable systems for indirect visual SLAM were already available around the year 2000. To date, indirect methods are the preferred approach for real-time robot vision on platforms with limited compute. Moreover, once the intermediate representation is determined, the subsequent computations are often mathematically simpler, making the resulting inference problems more tractable. In the case of visual SLAM, for example, once a set of corresponding points is identified across a set of images, the resulting problem of localization and mapping amounts to the classical bundle adjustment (BA) problem for which a multitude of powerful solvers and approximation methods exist.

In turn, direct methods have the potential to provide superior accuracy because they make use of all available input information. While the processing of all available input information (for example all pixels in each image) is computationally cumbersome and capturing the complex relationship between the quantities of interest (localization and mapping) and the raw input data (e.g., the brightness of each pixel) may create additional non-convexities in the loss function used for estimation, there exist eficient approximation and inference strategies with first real-time capable methods for direct visual SLAM emerging in the 2010s. As we will see in Part II and III, the eficient processing of huge amounts of input data can be facilitated by using graphics processing units (GPUs) to parallelize computations.

In both direct and indirect methods the measurements are used to infer the robot pose and map representation. There is a well-established literature in estimation theory describing how we can infer quantities of interest (in our case, the robot poses and the surrounding map) from observations. This book particularly focuses on estimation theoretic tools —reviewed and tailored to the SLAM problem in Chapter 1 and Chapter 2— that have their foundations in probabilistic inference and that rephrase estimation in terms of solving optimization problems.

![](images/aac7fe9f82ae4f7d1a14bfeb3397f41fcb4c287a4cef872534c48374bb46a97b.jpg)  
Figure I.2 Typical indirect methods for SLAM include a front-end (to process sensing data into a more manageable representation and to detect loop closures) and a back-end (to estimate the robot’s pose and a geometric map). The back-end often has a number of helper modules aimed at helping with robustness, computational tractability, and map quality.

Indirect methods induce a natural split in common SLAM architectures (Figure

I.2): the raw sensor data is first passed to a set of algorithms (the SLAM frontend) in charge of extracting intermediate representations; then such intermediate representations are passed to an estimator (the SLAM back-end), that estimates the quantities of interest. The front-end is typically also in charge of building an initial guess: this is an initial estimate the back-end can use for iterative optimization, hence mitigating convergence issues due to non-convexity. Let us discuss a few examples to clarify the diference between the SLAM front-end and back-end.

![](images/59e887d481db0f93e79c752362d862470d5f0ac715d961f33a253ecef7a7fca1.jpg)

![](images/7b4d2e535e099015d46d464a7f63f1439cdd1828c13da6d4ef38cecd85afb001.jpg)  
(b)  
Figure I.3 (a) In landmark-based SLAM models, the front-end produces measurements to 3D landmarks and the back-end estimates the robot trajectory (as a set of poses) and landmark positions. (b) In pose-graph-based SLAM models, the front-end abstracts the raw sensor measurements in terms of odometry and loop closure measurements (these are typically relative pose measurements) and the back-end estimates the overall robot trajectory.

Example I.1 (Visual SLAM: from pixels to landmarks) Visual SLAM uses camera images to estimate the robot trajectory and a sparse 3D point cloud map. The typical front-end of a visual SLAM system extracts 2D keypoints in each image and matches them across frames such that each group (a feature track) corresponds to re-observations of the same 3D point (a landmark) across diferent camera views. The front-end also computes rough estimates of the camera poses and 3D landmark positions by using computer vision techniques known as minimal solvers.<sup>1</sup> Then, the back-end is in charge of estimating (or refining) the unknown 3D position of the landmarks and the robot poses observing them by solving an optimization problem, known as bundle adjustment. This example leads to a landmark-based (or featurebased) SLAM model, visualized in Figure I.3(a). We will discuss visual SLAM at length in Chapter 7.

Example I.2 (LiDAR SLAM: from scans to odometry and loop closures) LiDAR SLAM uses LiDAR scans to estimate the robot trajectory and a map. A common front-end for LiDAR SLAM consists in using scan matching algorithms (e.g., the Iterative Closest Point or ICP) to compute the relative pose between two LiDAR scans. In particular, the front-end will match scans taken at consecutive time instants to estimate the relative motion of the robot between them (the so called odometry) and will also match scans corresponding to multiple visits to the same place (the so called loop closures). Odometry and loop closure measurements are then passed to the back-end that optimizes the robot trajectory by solving an optimization problem, known as pose-graph optimization (PGO). This example leads to a pose-graph-based SLAM model, visualized in Figure I.3(b). We discuss LiDAR SLAM in Chapter 8.

The previous examples showcase three popular examples of “intermediate representations” (or pseudo-measurements) that are produced by the front-end and passed to the back-end (Figure I.2): landmark observations, odometry, and loop closures. In complex SLAM systems, these representations can be used in combination: for instance, in certain visual-SLAM systems one might extract keypoints corresponding to 3D landmarks, and further process them to compute relative poses corresponding to odometry and loop closures, and finally use a pose-graph-based back-end. The choice of the front-end/back-end split is about selecting a desired trade-of between computation and accuracy. Extracting simpler representations might lead to much faster back-end solvers (e.g., performing pose-graph optimization is typically much faster than doing BA); but at the same time abstracting measurements induces approximation in how the measurements are modeled in the back-end, hence leading to small inaccuracies (e.g., BA is typically more accurate than pose-graph optimization).

We remark that loop closures are a key aspect of SLAM. If we only use odometry for trajectory estimation, the resulting estimate —obtained by accumulating odometry motion estimates— is bound to drift over time, leading to severe distorsion in the trajectory estimate. Revisiting already visited places is crucial to keep the trajectory estimation error bounded and obtain globally consistent maps. We also remark that loop closures are implicitly captured in landmark-based SLAM, where loop closures correspond to new observations of previously seen landmarks.

We conclude this section by observing how SLAM research cuts across multiple disciplines. The SLAM front-end extracts features from raw sensor data, hence touching disciplines ranging from signal processing, geometry, 2D computer vision, and machine learning. The SLAM back-end performs estimation given measurements from the front-end, hence touching estimation theory, optimization, and applied mathematics. This variety of ideas and influences contribute to making SLAM a fascinating and multi-faceted problem.

![](images/410e04eeb348a167f251ee4a7548fc7cbff9604206e516c8475747e530e0b499.jpg)  
Figure I.4 SLAM plays an important role in the overall autonomy pipeline of a robot that interacts with the world, and provides necessary information for control and motion planning.

## I.3 The Role of SLAM in the Autonomy Architecture

The role of SLAM is to serve downstream tasks. For instance, the robot-pose estimate can be used to control the robot to follow a desired trajectory, while the map (in combination with the current robot pose) can be used for motion planning (Figure I.4). Here motion planning is used in a broad sense: while SLAM is typically used to build large-scale maps to support navigation tasks, it can also support building local 3D maps to enable manipulation and grasping.

While it would be tempting to think about SLAM as a monolithic system that takes sensor data in input and instantaneously outputs robot poses and map, the actual implementation of these systems and their integration in autonomy architectures is more complicated in practice. This is due to the fact that the robot needs to close diferent control and decision-making loops with diferent latency requirements. For instance, with reference to Figure I.4, the robot will need to close low-level control loops over its trajectory (this is the standard feedback control loop at the top-right of the figure), which might require relatively high rates and low-latency to be stable; for instance, a UAV flying at high speed might need the front-end to produce odometry estimates with a latency of a few milliseconds. On the other hand, closing the loop over motion planning (the outer loop in Figure I.4) can accommodate higher latencies, since global planning typically runs at lower rates; hence it might be acceptable for the back-end to provide global trajectory and map estimates with a latency of seconds. For these reasons, a typical implementation of a SLAM system involves multiple processes running in parallel and in a way that slower processes (e.g., global pose and map optimization in the back-end) do not get in the way of faster processes (e.g., odometry estimation).

We also observe that the processes involved in a SLAM system have complex interactions (as emphasized by the bi-directional edges in Figure I.4): for instance, while the front-end feeds the odometry to the back-end, the back-end periodically applies global corrections to the odometric trajectory, which is then passed to the motion controller; similarly, while the front-end computes loop closures that are fed to the back-end, the back-end can also inform loop closure detection about plausible or implausible loop closure opportunities.

While the SLAM back-end might run at a slower pace, it is important to emphasize that it must remain online: it is desirable for the overall SLAM system to have a reasonable runtime that does not grow unbounded over time, and can be achieved on embedded robotics hardware as data streams are collected. These realtime constraints are vital for the robot to properly act in a complex environment, in particular with faster robots such as drones. This can be considered a main feature that historically diferentiated SLAM from related problems in computer vision, such as structure from motion (SFM): while SFM is also used to reconstruct the geometry of a 3D scene from camera images, it often relies on powerful computers (e.g., a cluster of servers as in [13]), its runtime can be in the order of hours, and it is commonly applied to an unordered dataset of images. On the other hand, SLAM involves data causally collected by the robot over time as the robot explores the environment, and must run in seconds in the face of tight computational constraints. It is worth emphasizing that the boundaries between visual SLAM and SFM have become increasingly blurry, also thanks to work on online SFM in vision (going back to the early 2000s, e.g., [524, 209]), and the use of SLAM methods for postprocessing datasets ofline. Therefore many researchers might use visual SLAM and SFM as interchangeable terms.

## I.3.1 Do We Really Need SLAM for Robotics?

From our description above, SLAM feels like an intriguing but very challenging problem, ranging from its complex implementation, to the need of fast runtime on resource-constrained platforms. Therefore, a fair question to ask is whether we can develop complex autonomous robots that do not rely on SLAM. We refine this question into three sub-questions.

Q1. Do we need SLAM for any robotics task? We started this section stating that SLAM is designed to support robotics tasks. Then a natural question is whether it is necessary for any robotics task. The answer is clearly: no. More reactive tasks, for instance keeping a target in sight, can be solved with simpler control strategies (e.g., visual servoing).<sup>2</sup> Similarly, if the robot has to operate over small distances, relying on odometry estimates and local mapping might be acceptable. Moreover, if the environment the robot operates in has some infrastructure for localization, then we may not need to solve SLAM. Nevertheless, SLAM seems an indispensable component for long-term robot operation in unstructured (i.e., infrastructure-free) environments: long-term operation typically requires memory (e.g., to go back to previously seen objects or find suitable collision-free paths), and map representations built from SLAM provide such a long-term memory.

![](images/e5190e0af9c54cda429bb80e5995e6c420566ee762630c141193599664bb1b22.jpg)

![](images/cd664f6db02c3f128d91cc5364960f24370e88c3ab368f53c0311a5ab6beb32b.jpg)

![](images/978fd7d0ca03804df6fcae141d06f118dae06713a1f8fddd55ca7c2a8e804341.jpg)

![](images/8234a9e9bff006acd6de2e3b7b246277087410155c9e42014c73fccd15cbb6d1.jpg)  
Figure I.5 (a) Our robot visits Ofice 1 in a building and then —after exploring other areas (including Ofice 2 and the Kitchen)— it visits Ofice 3, which is just next door from Ofice 1. Obstacles are shown in black and ground-truth trajectory is shown in green. (b) Odometric estimate of the trajectory, labeled with corresponding room labels. (c) Groundtruth topological map of the environment. (d) Estimated topological map in the presence of perceptual aliasing, causing the robot to think that Ofice 1 and 3 are the same room.

Q2. Do we need globally consistent geometric maps for navigation? A major focus in SLAM is to optimize the trajectory and map representations such that they are metrically accurate (or globally consistent) — this is precisely the role of the SLAM back-end. One might ask whether metric accuracy is actually needed. One alternative that comes to mind is to just use odometry to get locally consistent trajectory and map estimates; this circumvents the need for loop closures and back-end optimization. Unfortunately, due to its drift, odometry is unsuitable to support long-term operation: imagine that our robot visits Ofice 1 in a building and then, after exploring other areas of the building, it visits Ofice 3, which is just next door from Ofice 1 (see Figure I.5(a)). Using just odometry, the robot might be misled to conclude that Ofice 1 and Ofice 3 are quite far from each other (due to the odometry drift), hence being unable to realize there is a short path connecting the two ofices (Figure I.5(b)). A slightly more sophisticated alternative is to build a topological map instead. A topological map can be thought of as a graph where nodes are places the robot visited and edges represent traversability between the places connected by each edge (Figure I.5(c)). The diference with the metric SLAM lens we adopt in this handbook is that nodes and edges in a topological map do not carry metric information (distances, bearing, positions), hence they do not require any optimization: one can simply add edges to a topological map when the robot traveled between two places (odometry) or when a place recognition module recognizes the places to overlap (loop closures). While this seems a perfectly reasonable approach, the main issue is that place recognition techniques are not perfect and, more fundamentally, two diferent places might look similar (a phenomenon known as perceptual aliasing). Therefore, going back to our example above, if Ofice 1 and Ofice 3 look very similar, a purely topological approach might be misled to think there is a single ofice instead (Figure I.5(d)). On the other hand, metric SLAM approaches can use geometric information to conclude that the two ofices are indeed two diferent rooms, by giving the user access to a more powerful set of tools to decide whether place recognition results are correct and if two observations correspond to the same place; we will discuss these tools at length in Chapter 3.

Q3. Do we need maps? SLAM builds a map that can be directly queried, inspected, and visualized. As we will see in Chapter 5, there are many ways to represent a map, including 3D point clouds, voxels, meshes, neural radiance fields, and others. On the other hand, one might take a completely diferent approach: in order for the robot to execute a task, the robot might be trained to translate raw sensor data directly to actions (e.g., using Reinforcement Learning), hence circumventing the need to build a map. In such an approach, the neural network trained from sensor data to actions will arguably create an internal representation, but such an internal representation cannot be directly queried, inspected, or visualized. While the jury is still out on whether maps are indeed necessary, there is some initial evidence that using maps as an intermediate representation is at least beneficial in completing many visual tasks for robotics [912, 1252]. Moreover, maps have the benefit of being useful across a wide variety of tasks, while a representation that is fully learned in the context of a single task might not be able to support new unseen tasks. Finally, we observe that there are several applications where the goal is to have a map that can be inspected. This is the case in search-and-rescue robotics applications where it is desirable to provide a map to help first-responders. Moreover, it is the case for several applications beyond robotics (e.g., real-estate planning and visualization, construction monitoring, virtual and augmented reality), where the goal is for a human to inspect or visualize the map.

## I.4 Past, Present, and Future of SLAM, and Scope of this Handbook

The design of algorithms for spatial reasoning has been at the center-stage of robotics and computer vision research since their inception. At the same time, SLAM research keeps evolving and expanding to novel tools and problems.

## I.4.1 Short History and Scope of this Handbook

As discussed across the various chapters of this book, SLAM has multiple facets. As a consequence, its history is also multi-faceted with origins that can be traced back across diferent scientific communities.

Creating maps of the world from observations and measurements is among the oldest challenges in history and leads to the fields of geodesy (the science measuring properties of the Earth) and surveying. There are many pioneers who contributed to this field. Carl Friedrich Gauss triangulated the Kingdom of Hannover in the years 1821-1825. Sir George Everest served as Surveyor General of India 1830-1843 in the Great Trigonometric Survey, eforts for which he was honored by having the world’s highest mountain named after him. In 1856, Carl Maximilian von Bauernfeind published a standard book on “Elements of Surveying” [60]. He subsequently founded the Technical University of Munich in 1868 with a central focus on establishing geodesy as a scientific discipline. Andr´e-Louis Cholesky developed the well-known Cholesky matrix decomposition while surveying Crete and North Africa before the First World War.

The problem of visual SLAM is also closely related to the field of photogrammetry and the problems of Structure from Motion, whose origins can be traced back to the 19th century. We discuss this further in Chapter 7.

In robotics, the origin of SLAM is typically traced back to the seminal work of Smith and Chessman [1018] and Durrant-Whyte [293], as well as the parallel work by Crowley [233] and Chatila and Laumond [172]. The acronym SLAM was coined in 1995, as part of the survey paper [294]. These early works developed two fundamental insights. The first insight is that to avoid drift in unknown environments, one needs to simultaneously estimate the robot poses and the position of fixed external entities (e.g., landmarks). The second insight is that existing tools from estimation theory, and in particular the celebrated Extended Kalman Filter (EKF), could be used to perform estimation over an extended state describing the robot poses and the landmark positions, leading to a family of EKF-SLAM approaches.

EKF-SLAM approaches have been extremely popular but face three main issues in practice. The first is that they are sensitive to outliers and data association errors. These errors may result from failures of place recognition or object detection, where the robot believes it is observing a given object or place, but it is actually observing a diferent (but possibly similarly looking) one. If these spurious measurements are not properly handled, EKF-SLAM produces grossly incorrect estimates.

![](images/39aa3c350539014b6a42e0dfc41f6e5b9930402ba9a97786864f3a461482437c.jpg)

![](images/9e2e402426418696fea21df8bbcceff532d7dd2c4c6c0c04ab509b78392d0534.jpg)

![](images/71c09f04049a7e2bfa371c3db1fa1f76ccf84cbd4768e36e126548587a3226ba.jpg)

![](images/f720ee60bdc04ddda6bfbea95113e29d596d4b3c2a7f19610c66d5a16624914f.jpg)

![](images/328a258dfadf74cd3ad47db541eace355f341a6d75e4fbfa52a1c663d7658f0d.jpg)  
Early Graph SLAM (F. Lu / E. Milios, 1997)

![](images/1cc897c63b51b9738ed301a7bee1123296eb1feef2932d390e00d641da908bfc.jpg)  
Compressed EKE SL AM Victoria Park Dataset (credit: J. Guivant / E. Nebot, 2001)

![](images/d1c5f524785b06f31f6202391f0c117b63d288bf7e7613e05d95c9222b365836.jpg)  
Rao-Blackwellized Particle Filte<sup>(a)</sup> <sup>Trajectory</sup> <sup>based</sup> <sup>on</sup> <sup>odometry</sup> <sup>only.</sup> SLAM (credit: G. Grisseti / C. Stachniss / W. Burgard, 2007)r step, while reordering the variables every

![](images/a7bd4b91824843e038cf47273d07aa262d1d15afef1f0b9925fe0da3c8174502.jpg)  
<sup>(b)</sup> <sup>Final</sup> <sup>trajectory</sup> <sup>and</sup> <sup>evidence</sup> <sup>grid</sup> <sup>map.</sup> Incremental Smoothing and Mapping t. iSAM calculates the full solution for <sub>9</sub>(iSAM) (credit: M. Kaess / A. The problem has 910 ⇥ 3 = 2730 variables Ranganathan / F. Dellaert, 2008)

![](images/c2b1360e119cd09e148e8b46b44203dfa0f48503c4c766b4dae69ba4a3e56d00.jpg)  
Large-Scale Direct SLAM<sup>r srte</sup> l <sup>h s</sup> (credit: J.Engel / T. Schöps <sup>a</sup>s <sup>s</sup>ein E<sub>n</sub> <sup>s</sup>iLC. w <sup>t</sup>h D. Cremers, 2014)e<sup>f</sup>r2 el <sup>n</sup>ifi<sup>D</sup>-<sup>R</sup>e <sup>e</sup>  p

![](images/064f4a1a4455e88d3c6fe225a26ca21e175a4ebaca6ce79d5eed79598454bfdb.jpg)  
ORB-SLAM (credit: R. Mur <sup>t</sup>ra <sup>L</sup>S <sup>s</sup>e <sup>d</sup>i <sup>c</sup>a<sup>f</sup>r <sup>a</sup>c in<sup>w r</sup>e <sup>R S</sup>L <sup>T</sup>hArtal, J Montiel, J Tardós, 2015)

![](images/6b30dd3c6297f215d6c34518ad4a18e2d24f0f4695abb682dfe6c1b4506482c3.jpg)  
contains 52 414 entries for 5823 variables, w      3D SLAM in the DARPA SubT Competition (credit: K. Ebadi et al., 2023)

<sub>Fig. 11. iSAM results for the MIT Killian Court dataset. iSAM calculates the full solution for the</sub><sup>nd</sup> t <sup>h</sup>ibi<sub>t</sub> <sup>h</sup>i<sub>g</sub> <sup>tru</sup>c<sub>t</sub> i<sup>s</sup> <sup>on</sup> <sup>ac</sup>cu <sup>d</sup>e <sup>to</sup>ri<sup>te</sup>x<sup>fro</sup>m <sup>me</sup>n <sup>nt</sup> <sub>a</sub> <sup>as</sup>u <sup>p</sup> o<sub>p</sub> <sup>O</sup>n <sup>ar</sup>l<sub>y</sub> <sup>r</sup>i<sup>s</sup>in. <sup>A</sup><sub>n</sub><sup>s</sup> r<sub>o</sub><sup>es</sup>k <sup>ce</sup>d<sup>es</sup>, <sup>uc</sup>e] re<sup>fr</sup>2 <sup>on</sup> i <sup>an</sup>d <sup>no</sup>t <sup>tw</sup>o <sup>s</sup>ec <sup>a</sup>rk<sup>h</sup> P <sup>t</sup>h<sub>e</sub> <sup>at</sup>io<sub>n</sub>Figure I.6 The history of SLAM is filled with numerous advances that have led to modern <sup>e</sup> s<sub>e</sub> <sup>ss</sup> r <sup>In</sup> t <sup>s</sup> f<sub>r</sub> <sup>c</sup>ce ot<sup>y</sup> Oi<sup>n</sup>g w<sub>h</sub>n<sub>e</sub><sup>ot</sup>i<sub>o</sub> <sup>w</sup>hi <sup>jo</sup>i<sub>n</sub> <sup>e</sup>n<sub>t</sub> iz<sub>a</sub> <sup>f</sup> th <sup>o</sup>r<sub>e</sub> <sup>e</sup>su<sup>e</sup>r i<sup>st</sup> t<sup>th</sup> <sup>t</sup> R <sup>a</sup>li<sub>g</sub><sup>e</sup> e<sup>te</sup>dwi<sup>e</sup> <sub>T</sub> <sup>h</sup>a <sup>e</sup> el<sup>o</sup>c<sup>ds</sup> <sub>o</sub> th<sup>M</sup> <sup>c</sup>o<sub>v</sub> i<sup>t</sup>h SLAM systems capable of localizing and mapping robots in challenging real-world envi-<sup>en</sup>c <sup>s</sup>tn <sup>re</sup>st <sup>L</sup>S <sup>b</sup>le <sup>s</sup> t<sup>-</sup>S<sup>r</sup>ge Owii<sup>s</sup> t <sup>o</sup>p <sup>re</sup> <sup>n</sup> t <sup>o</sup>ss <sup>c</sup>cu is<sup>2</sup>].<sup>re</sup>s<sup>y</sup>na<sup>so</sup>n <sup>D</sup>-<sup>g</sup> <sup>r</sup>. F<sup>at</sup> P<sup>an</sup> <sup>M</sup> R <sup>be</sup>e <sup>g</sup>h<sup>a</sup>ti<sup>he</sup> s <sup>r</sup>st <sup>c</sup>o <sup>d</sup> <sup>ry</sup> ronments. This image shows a selection of representative highlights. Figures from [701] <sup>re</sup>sp <sup>t</sup>h <sup>t</sup>he <sup>S</sup>L <sup>W</sup>e <sup>c</sup>or<sup>M</sup> <sup>oo</sup>p <sup>-</sup>SL<sup>o</sup>op  <sup>g</sup>ol ize <sup>ca</sup>rd <sup>a</sup> p <sup>e</sup> c <sup>e</sup> t <sup>t</sup> b<sup>e</sup> fu<sup>g</sup> re<sup>c</sup> o<sup>d</sup> f<sup>A</sup>M trallyM<sup>S</sup>E-D <sup>n</sup>ab <sup>ta</sup>il <sup>e</sup>xp<sup>ue</sup>n <sup>p</sup>er<sup>a</sup>ris <sup>e</sup>s. <sup>c</sup>es(©1997 Springer), [413] (©2001 IEEE), [399] (©2007 IEEE), and [787] (©2015 IEEE).

<sup>os</sup>i <sup>M</sup> a <sup>n</sup> th tw <sup>wh</sup>ii<sup>m</sup>il<sup>ue</sup>n i<sup>g</sup>h<sup>ne</sup>a<sup>hm</sup> <sup>o</sup>v <sup>rf</sup>or <sup>za</sup>ti <sup>t</sup>h a <sup>O</sup>R <sup>re</sup>s<sup>L</sup>A<sup>ys</sup>te <sup>s</sup>ca <sup>Dd</sup> th <sup>t</sup>h<sup>we</sup>v<sup>he</sup>m <sup>es</sup>u <sup>T</sup>U m<sup>rf</sup>or <sup>sa</sup>m <sup>a</sup>l<sup>u</sup>a<sup>s</sup> tThe second issue is related to the fact that EKF relies on linearization of the equations describing the motion of the robot and sensor observations. In practice, the linearization point is typically built from odometry and when the latter drifts, the linearized system might be a poor approximation of the original nonlinear system. This leads EKF-SLAM to diverge when odometry accumulates substantial drift. The third problem is about computational complexity: a naive implementation of the Kalman Filter leads to a computational complexity that grows quadratically in the number of state variables, due to the need to manipulate a dense covariance matrix. In a landmark-based SLAM problem it is not uncommon to have thousands of landmark, which makes the naive approach prohibitive to run in real-time.

As a response to these issues, in the early 2000s, the community started focusing on particle-filter-based approaches [771, 1013, 399], which model the robot trajectory using a set of hypothesis (or particles), building on the theory of particle filtering in estimation theory.<sup>3</sup> When used in combination with landmark-based maps, these models allowed using a large number of landmarks (breaking through the quadratic complexity of the EKF); moreover, they allowed to more easily estimate dense map models, such as 2D occupancy grid maps. Also, these approaches did not rely on linearization and were less sensitive to outliers and incorrect data association. However, they still exhibited a trade-of between computation and accuracy: obtaining accurate trajectories and maps requires using many particles (in the thousands) but the more particles, the more computation. In particular, for a finite amount of particles, a particle filter may still diverge when none of the sampled particles are near the real trajectory of the robot (an issue known as particle depletion); this issue is exacerbated in 3D problems where one needs many particles to cover potential 3D poses of the robot.

Between 2005 and 2015, a key insight pushed to the spotlight an alternative approach to SLAM. The insight is that while the covariance matrix appearing in the EKF is dense, its inverse (the so called Information Matrix) is very sparse and has a very predictable sparsity pattern when past robot poses are retained in the estimation [313]; this allows designing filtering algorithms that have close-to-linear complexity, as opposed to the quadratic complexity of the EKF. While this insight was initially applied to EKF-like approaches, such as EIF, it also paved the way for optimization-based approaches. Optimization-based approaches were first proposed in the early days of SLAM [701], but then disregarded as too slow to be practical. The sparsity structure mentioned above allowed rethinking these optimization methods and making them more scalable and solvable in online fashion [249, 532].<sup>4</sup> This new wave can be interpreted as a shift toward yet another estimation framework: maximum likelihood and maximum a posteriori estimation. These frameworks rephrase estimation problems in terms of optimization, while describing the structure of the problem in terms of a probabilistic graphical model, or, specifically, a factor graph. The resulting factor-graph-based approach to SLAM is still the dominant paradigm today, and has also shaped the way the community thinks about related problems, such as visual and visual-inertial odometry. The optimization lens is a powerful one and allows a much deeper theoretical analysis than previously possible (see Chapter 6). Moreover, it is fairly easy to show that the EKF (with suitable linearization points) can be understood as a single iteration of a nonlinear optimization solver, hence making the optimization lens strictly more powerful than its filtering-based counterpart. Finally, the optimization-based perspective appears more suitable for recent extensions of SLAM (described in the next section and Part III of this handbook), where one wants to estimate both continuous variables (describing the scene geometry) and discrete variables (describing semantic aspects of the scene).

This short history review stops at 2015, while the goal of Part III of this handbook is to discuss more modern trends, including those triggered by the “deep learning revolution”, which started around 2012 and slowly permeated to robotics. We also remark that the short history above mostly gravitates around what we called the SLAM back-end (essentially, the estimation engine), while the development of the SLAM front-end traces back to work done across multiple communities, including computer vision, signal processing, and machine learning.

As a result of the considerations mentioned above, this handbook will primarily focus on the factor-graph-based formulation of SLAM. This is a decision about scope and does not detract from the value of ongoing works using other technical tools. For instance, at the time of writing of this handbook, EKF-based tools are still popular for visual-inertial odometry applications (building on the seminal work from Mourikis and Roumeliotis [777]), and novel estimation formulations have been developed, including invariant [56] and equivariant filters [331], as well as alternative formulations based on random finite sets [783].

## I.4.2 From SLAM to Spatial AI

SLAM essentially focuses on estimating geometric properties of the environment (and the robot). For instance, the SLAM map carries information about obstacles in the environment, distances and traversable paths between two locations, or geometric coordinates of distinctive landmarks. In this sense, SLAM is useful as a representation for the robot to understand and execute commands such as “robot: go to position $[ x , y , z ] ^ { \mathfrak { r } }$ , where $[ x , y , z ]$ are the coordinates (in the map frame) of a place or object the robot has to reach. However, specifying goals in terms of coordinates is not suitable for non-expert human users and it is definitely not the way we interact or specify goals for humans. Therefore, it would be desirable for the next generation of robots to understand and execute high-level commands specified in natural language, such as “robot: pick up the clothes in the bathroom, and take them to the laundry room”. Parsing these instructions requires the robot to understand both geometry (e.g., where is the bathroom) and semantics (e.g., what is a bathroom or laundry room, which objects are clothes) of the environment.

This realization has recently pushed the research community to think about SLAM as an integrated component of a broader spatial perception system, that simultaneously reasons about geometric, semantic, and possibly physical aspects of the scene, in order to build a multi-faceted map representation (a “world model”), that enables the robot to understand and execute complex instructions. The resulting Spatial AI algorithms and systems have the potential to increase robot autonomy and have rapidly progressed over the last decade. Intuitively, one can think that Spatial AI has SLAM as a submodule (to handle the geometric reasoning part), but provides extra semantic reasoning capabilities. This allows closing the loop over task planning, as shown in Figure I.7, where now the robot can take high-level semantic goals instead of coordinates of motion goals. We will discuss Spatial AI at length in Part III of this handbook.

![](images/f196b7a3a162da62e811f7a60f9496a2245f71009e54d93a6e7140d0551f8a21.jpg)  
Figure I.7 Spatial AI (or spatial perception) extends the geometric reasoning capabilities of SLAM to also perform semantic and physical reasoning. While the SLAM block is informed by odometry and provides a geometric understanding of the scene, the Spatial AI block is informed by the SLAM results and adds a scene understanding component, spanning semantics, afordances, dynamics, and more. This allows closing the loop over higher-level decision making modules, such as task planning, and allows the user to specify higher-level goals the robot has to achieve.

## I.5 Handbook Structure

The chapters of this handbook are grouped into three parts.

Part I covers the foundations of SLAM, with particular focus on the estimationtheoretic machinery used in the SLAM back-end and the diferent types of map representations SLAM can produce. In particular, Chapter 1 introduces the factorgraph formulation of SLAM and reviews how to solve it via iterative nonlinear optimization methods. Then, Chapter 2 takes the indispensable step of extending the formulation to allow the estimation of variables belonging to smooth manifolds, such as rotation and poses. Chapter 3 discusses how to model and mitigate the impact of outliers and incorrect data association in the SLAM back-end. Chapter 4 reviews techniques to make the back-end optimization diferentiable, a key step towards interfacing traditional SLAM methods with more recent deep learning architectures. Chapter 5 shifts the focus from the back-end to the question of dense map representations and discusses the most important representations used for SLAM. Finally, Chapter 6 discusses more advanced solvers and theoretical properties of the SLAM back-end.

Part II covers the “state of practice” in SLAM by discussing key approaches and applications of SLAM using diferent sensing modalities. This part touches on the

SLAM front-end design (which is heavily sensor dependent) and exposes what’s feasible with modern SLAM algorithms and systems. Chapter 7 reviews the large body of literature on visual SLAM. Chapter 8 and Chapter 9 cover LiDAR SLAM and radar SLAM, respectively. Chapter 10 discusses recent work on SLAM using event-based cameras. Chapter 11 reviews how to model inertial measurements as part of a factor-graph SLAM system and discusses fundamental limits (e.g., observability). Chapter 12 discussed how to model other sources of odometry information, including wheel and legged odometry.

Part III provides a future-looking view of the state of the art and recent trends. In particular, we touch on a variety of topics, ranging from computational architectures, to novel problems and representations, to the role of language and Foundation Models in SLAM. In particular, Chapter 13 reviews recent improvements obtained by introducing deep learning modules in conjunction with diferentiable optimization in SLAM. Chapter 14 discusses opportunities and challenges in using novel map presentations, including neural radiance fields (NeRFs) and Gaussian Splatting. Chapter 15 covers recent work on SLAM in highly dynamic and deformable environments, touching on real applications from mapping in crowded environments to surgical robotics. Chapter 16 discusses progress in Spatial AI and metric-semantic map representations. Chapter 17 considers new opportunities arising from the use of Foundation Models (e.g., Large Vision-Language Models) and their role in creating novel map representation for Spatial AI that allow understanding and grounding “open-vocabulary” commands given in natural language. Finally, Chapter 18 focuses on future computational architectures for Spatial AI that could leverage more flexible and distributed computing hardware and better support spatial perception across many robotic platforms.

Factor Graphs for SLAM Frank Dellaert, Michael Kaess, and Timothy Barfoot

In this chapter we introduce factor graphs and establish the connection with maximum a posteriori (MAP) inference and least squares for the case of Gaussian priors and Gaussian measurement noise. We focus on the SLAM back-end, after measurements have been extracted by the front-end and data association has been accomplished. We begin by visualizing SLAM problems using factor graphs (Section 1.1), then show how MAP inference leads to least squares optimization (Section 1.2). We discuss methods for solving both linear (Section 1.3) and nonlinear optimization problems (Section 1.4), and then make the connection between sparsity, factor graphs, and Bayes nets more explicit (Section 1.5). We explore the variable elimination algorithm and its graphical interpretation (Section 1.6), and finally apply this to develop the Bayes tree and the incremental smoothing and mapping (iSAM) algorithm (Section 1.7). The chapter concludes with a discussion of further readings and recent trends (Section 1.8).

## Historical Note

A smoothing approach to SLAM involves not just the most current robot location, but the entire robot trajectory up to the current time. A number of authors consider the problem of smoothing the robot trajectory only [172, 701, 700, 420, 594, 312], now known as pose-based SLAM. This is particularly suited to sensors such as laser-range finders that yield pairwise constraints between nearby robot poses.

More generally, one can consider the full SLAM problem [1088], i.e., the problem of optimally estimating the entire set of sensor poses along with the parameters of all features in the environment. From a computational view, this optimizationbased smoothing approach was recognized as beneficial since (a) in contrast to the filtering-based covariance or information matrices, which both become fully dense over time [849, 1087], the information matrix associated with smoothing is and stays sparse; (b) in typical mapping scenarios (i.e., not repeatedly traversing a small environment) this matrix is a much more compact representation of the map covariance structure. This led to a flurry of work between 2000 and 2005 where these ideas were applied in the context of SLAM [287, 340, 339, 1088].

Square-root smoothing and mapping (SAM), also known as the ‘factor-graph approach’, was introduced in [249, 254] based on the fact that the information matrix or measurement Jacobian can be eficiently factorized using sparse Cholesky or QR factorization, respectively. This yields a square-root information matrix that can be used to immediately obtain the optimal robot trajectory and map. Factoring the information matrix is known in the sequential estimation literature as square-root information filtering (SRIF), and was developed in 1969 for use in JPL’s Mariner 10 missions to Venus [83]. The use of square roots results in more accurate and stable algorithms, and, quoting Maybeck [742], “a number of practitioners have argued, with considerable logic, that square root filters should always be adopted in preference to the standard Kalman filter recursion”.

Below we discuss in detail how factor graphs are a natural representation for the sparsity inherent in SLAM problems, how (sparse) matrix factorization into a matrix-square root is at the heart of solving these problems, and finally how all this relates to the much more general variable elimination algorithm. Much of this chapter is an abridged version of a longer article by Dellaert et al. [255].

## 1.1 Visualizing SLAM With Factor Graphs

In this section we introduce factor graphs as a way of intuitively visualizing the sparse nature of the SLAM problem by first considering a toy example and its factor graph representation. We then show how many diferent flavors of SLAM can be represented as such, and how even in larger problems the sparse nature of many sparse problems is immediately apparent.

## 1.1.1 A Toy Example

We begin by examining a simple SLAM scenario to illustrate how factor graphs are constructed. Figure 1.1 shows a simple toy example illustrating the structure of the problem graphically. A robot moving across three successive poses ${ \mathbf { \mathit { p } } } _ { 1 } , { \mathbf { \mathit { p } } } _ { 2 }$ , and p<sub>3</sub> makes bearing observations on two landmarks $\ell _ { 1 }$ and $\ell _ { 2 }$ . To anchor the solution in space, let us also assume there is an absolute position/orientation measurement on the first pose ${ \pmb p } _ { 1 }$ . Without this there would be no information about absolute position, as bearing measurements are all relative.<sup>1</sup>

Because of measurement uncertainty, we cannot hope to recover the true state of the world, but we can obtain a probabilistic description of what can be inferred from the measurements. In the Bayesian probability framework, we use the language of probability theory to assign a subjective degree of belief to uncertain events. We do this using probability density functions $( P D F s ) p ( { \pmb x } )$ over the unknown variables x. PDFs are non-negative functions satisfying

![](images/7f2cd7879950afca0abfe3a2c35a1a703e4d6c8e28ee44484e2aef43386318b3.jpg)  
Figure 1.1 A toy simultaneous localization and mapping (SLAM) example with three robot poses and two landmarks. Above we schematically indicate the robot motion with arrows, while the dotted lines indicate bearing measurements.

$$
\int p (\boldsymbol {x}) \mathrm{d} \boldsymbol {x} = 1,\tag{1.1}
$$

which is the axiom of total probability. In the simple example of Figure 1.1, the state, $^ { x , }$ is

$$
\boldsymbol {x} = \left[ \begin{array}{c} \boldsymbol {p} _ {1} \\ \boldsymbol {p} _ {2} \\ \boldsymbol {p} _ {3} \\ \boldsymbol {\ell} _ {1} \\ \boldsymbol {\ell} _ {2} \end{array} \right],\tag{1.2}
$$

which is just a stacking of the individual unknowns.

In SLAM we want to characterize our knowledge about the unknowns x, in this case robot poses and the unknown landmark positions, when given a set of observed measurements z. Using the language of Bayesian probability, this is simply the conditional density or posterior

$$
p (\boldsymbol {x} | \boldsymbol {z}),\tag{1.3}
$$

and obtaining a description like this is called probabilistic inference. A prerequisite is to first specify a probabilistic model for the variables of interest and how they give rise to (uncertain) measurements. This is where probabilistic graphical models enter the picture.

Probabilistic graphical models provide a mechanism to compactly describe complex probability densities by exploiting the structure in them [592]. Bayesian networks are perhaps the most well-known graphical model, consisting of variables nodes, each associated with a prior or conditional probability density. In Figure 1.2 we show the Bayesian network associated with the toy example of Figure 1.1, now showing an initial measurement $z _ { 1 }$ , dependent on the first pose $\pmb { p } _ { 1 }$ , and bearing measurements $z _ { \mathrm { 2 } } \ldots z _ { 4 }$ , each associated with both a pose and a landmark. It is convention to indicate known quantities in a Bayesian network as square nodes, as shown in the figure. However, while Bayesian networks are excellent for modeling, we next introduce a diferent graphical model is geared towards optimization, focuses on the unknown variables in the problem, exclusively.

![](images/de0dd895a761a6a8ef2799623bc1231feeeca13f4f5f7a71c2097acf985dfe2a.jpg)  
Figure 1.2 Bayesian network for the example in Figure 1.1, explicitly showing known measurements $z _ { 1 } \ldots z _ { 4 }$ as square nodes.

## 1.1.2 A Factor-Graph View

Because of locality, high-dimensional probability densities can often be factorized as a product of many factors, each of which is a probability density over a much smaller domain. Factor graphs are probabilistic graphical models that allow us to specify any density as a product of factors.

To motivate factor graphs, consider performing inference for the toy SLAM example. The posterior $p ( { \pmb x } | z )$ from (1.3) can be re-written using Bayes’ law, $p ( { \pmb x } | { \pmb z } ) \propto p ( { \pmb z } | { \pmb x } ) p ( { \pmb x } )$ , as

$$
p (\pmb {x} | \pmb {z}) \propto p (\pmb {p} _ {1}) p (\pmb {p} _ {2} | \pmb {p} _ {1}) p (\pmb {p} _ {3} | \pmb {p} _ {2})\tag{1.4a}
$$

$$
\times p (\pmb {\ell} _ {1}) p (\pmb {\ell} _ {2})\tag{1.4b}
$$

$$
\times p (\boldsymbol {z} _ {1} | \boldsymbol {p} _ {1})\tag{1.4c}
$$

$$
\times p (\boldsymbol {z} _ {2} | \boldsymbol {p} _ {1}, \ell_ {1}) p (\boldsymbol {z} _ {3} | \boldsymbol {p} _ {2}, \ell_ {1}) p (\boldsymbol {z} _ {4} | \boldsymbol {p} _ {3}, \ell_ {2}).\tag{1.4d}
$$

where we assumed a typical Markov chain generative model for the pose trajectory. Each of the factors represents one piece of information about the unknowns, x.

To visualize this factorization, we use a factor graph. Figure 1.3 shows the corresponding factor graph by example: all unknown states x, both poses and landmarks, have a node associated with them. Measurements are not represented explicitly as they are known, and hence not of interest. In factor graphs we explicitly introduce an additional node type to represent every factor in the posterior $p ( { \pmb x } | z )$ . In the figure, each small black node represents a factor, and—importantly—is connected to only those state variables of which it is a function. For example, the factor $\phi _ { 9 } ( p _ { 3 } , \ell _ { 2 } )$ is connected only to the variable nodes $\mathbf { \mathit { p } } _ { 3 }$ and $\ell _ { 2 }$ . In more detail, we have

![](images/0d4454ba5350193460ee9f212f491ce8f8f7890456553f8baea8c7f096294afa.jpg)  
Figure 1.3 Factor graph resulting from the example in Figure 1.1.

$$
\phi (\boldsymbol {p} _ {1}, \boldsymbol {p} _ {2}, \boldsymbol {p} _ {3}, \ell_ {1}, \ell_ {2}) = \phi_ {1} (\boldsymbol {p} _ {1}) \phi_ {2} (\boldsymbol {p} _ {2}, \boldsymbol {p} _ {1}) \phi_ {3} (\boldsymbol {p} _ {3}, \boldsymbol {p} _ {2})\tag{1.5a}
$$

$$
\times \phi_ {4} (\pmb {\ell} _ {1}) \phi_ {5} (\pmb {\ell} _ {2})\tag{1.5b}
$$

$$
\times \phi_ {6} (\pmb {p} _ {1})\tag{1.5c}
$$

$$
\times \phi_ {7} (\boldsymbol {p} _ {1}, \boldsymbol {\ell} _ {1}) \phi_ {8} (\boldsymbol {p} _ {2}, \boldsymbol {\ell} _ {1}) \phi_ {9} (\boldsymbol {p} _ {3}, \boldsymbol {\ell} _ {2}),\tag{1.5d}
$$

where the correspondence between the factors and the original probability densities in (1.4a)-(1.4d) should be obvious.

The factor values need only be proportional to the corresponding probability densities: any normalization constants that do not depend on the state variables may be omitted without consequence. Also, in this example, all factors above came either from a prior, e.g., $\phi _ { 1 } ( { p } _ { 1 } ) \propto p ( { p } _ { 1 } )$ or from a measurement, $\mathrm { e . g . } , \phi _ { 9 } ( p _ { 3 } , \ell _ { 2 } )$ α $p ( z _ { 4 } | p _ { 3 } , \ell _ { 2 } )$ . Although the measurement variables $z _ { 1 } \ldots z _ { 4 }$ are not explicitly shown in the factor graph, those factors are implicitly conditioned on them. Sometimes, when it helps to make this more explicit, factors can be written as (for example) ϕ<sub>9</sub> $( p _ { 3 } , \ell _ { 2 } ; z _ { 4 } )$ or even $\phi _ { z _ { 4 } } ( p _ { 3 } , \ell _ { 2 } )$

## 1.1.3 Factor Graphs as a Language

In addition to providing a formal basis for inference, factor graphs help visualize SLAM problems of many diferent flavors, give insight into the structure of the problem, and serve as a lingua franca that can help practitioners align across team boundaries. Each factor in a factor graph, such as those in Figure 1.3, can be thought of as an equation involving the variables it is connected to. There are typically many more equations than unknowns, which is why we need to quantify the uncertainty in both prior information and measurements. This will lead to a least squares formulation, appropriately fusing the information from multiple sources.

![](images/6a99785c0b3ee0e08985a7267ad8e41222afe130702e8ee950c0d95d9bc80ad6.jpg)  
Figure 1.4 A few variants of SLAM problems that can all be viewed through the factorgraph lens. Canonical landmark-based SLAM has both pose and landmark variables; landmarks are measured from poses and there is some motion prior between poses typically based on odometry. BA is the same but without the motion prior. PGO does not have landmark variables but enjoys extra loop closure measurements between poses. And, finally, STEAM is similar to landmark-based SLAM, but now poses can be replaced by higher-order states and a smooth continuous-time motion prior is used.

Many diferent flavors of the SLAM problem are all easily represented as factor graphs. Figure 1.1 is an example of landmark-based SLAM because it involves both pose and landmark variables. Figure 1.4 illustrates several other variants including bundle adjustment (BA) (same as landmark-based SLAM but without motion model), pose-graph optimization (PGO) (no landmark variables but includes loop closures between poses), and simultaneous trajectory estimation and mapping (STEAM) (poses are augmented to include derivatives such as velocity).

![](images/2d8cd5ffea6b9e14aa6cb1113efedd6424a226fa48a229db8350f93fde3f9e64.jpg)  
Figure 1.5 Factor graph for a larger, simulated SLAM example.

The factor graph for a more realistic landmark-based SLAM problem than the toy example could look something like Figure 1.5. This graph was created by simulating a 2D robot, moving in the plane for about 100 time steps, as it observes landmarks. For visualization purposes, each robot pose and landmark is rendered at its ground-truth position in 2D. With this, we see that the odometry factors form a prominent, chain-like backbone, whereas of to the sides binary<sup>2</sup> likelihood factors are connected to the 20 or so landmarks. All factors in such SLAM problems are typically nonlinear, except for priors.

Examining the factor graph reveals a great deal of structure by which we can gain insight into a particular instance of the SLAM problem. First, there are landmarks with a great deal of measurements, which we expect to be pinned down very well. Others have only a tenuous connection to the graph, and hence we expect them to be less well determined. For example, the lone landmark near the bottom right has only a single measurement associated with it.

## 1.2 From MAP Inference to Least Squares

Maximum a posteriori (MAP) inference is the process of determining the values for the unknowns x that maximally agree with the information present in the uncertain measurements and priors. In real life we are not given the ground-truth locations for the landmarks, nor the time-varying pose of the robot, although in many practical cases we might have a good initial estimate. Below we show that, given Gaussian measurement noise models and Gaussian priors, the optimization problem corresponding to MAP inference is nothing but the familiar nonlinear least squares problem.

## 1.2.1 Factor Graphs for MAP Inference

We are interested in the unknown state variables x, such as poses and/or landmarks, given the measurements z. The most-often-used estimator for these unknown state variables x is the maximum a posteriori (MAP) estimate, so named because it maximizes the posterior density $p ( { \pmb x } | { \pmb z } )$ of the states x given the measurements z:

$$
\boldsymbol {x} ^ {\mathrm{MAP}} = \underset {\boldsymbol {x}} {\arg \max} p (\boldsymbol {x} | \boldsymbol {z})\tag{1.6a}
$$

$$
= \arg \max _ {\boldsymbol {x}} \frac {p (\boldsymbol {z} | \boldsymbol {x}) p (\boldsymbol {x})}{p (\boldsymbol {z})}\tag{1.6b}
$$

$$
= \underset {\boldsymbol {x}} {\arg \max} p (\boldsymbol {z} | \boldsymbol {x}) p (\boldsymbol {x})\tag{1.6c}
$$

The second equation above is Bayes’ law, and expresses the posterior as the product of the measurement density $p ( \boldsymbol { z } | \boldsymbol { x } )$ and the prior $p ( { \pmb x } )$ over the states, appropriately normalized by the factor $p ( z )$ . The third equation drops the $p ( z )$ since this does not depend on the x and therefore will not impact the arg max operation. In other words, Equation (1.6c) tells us that the MAP estimate maximizes the product of the likelihood $p ( \boldsymbol { z } | \boldsymbol { x } )$ and the prior p(x).

We use factor graphs to express the unnormalized posterior $p ( \pmb { z } | \pmb { x } ) p ( \pmb { x } )$ . Formally a factor graph is a bipartite graph $\boldsymbol { F } = ( \boldsymbol { \mathcal { U } } , \boldsymbol { \mathcal { V } } , \boldsymbol { \mathcal { E } } )$ with two types of nodes: factors $\phi _ { i } \in \mathcal { U }$ and variables $\mathbf { \boldsymbol { x } } _ { j } \in \mathcal { V }$ . Edges $e _ { i j } \in \mathcal { E }$ are always between factor nodes and variables nodes. The set of variable nodes adjacent to a factor ϕ is written as $\mathcal { X } ( \phi _ { i } )$ ， and we write $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { i } }$ for an assignment to this set. With these definitions, a factor graph F defines the factorization of a global function $\phi ( { \pmb x } )$ as

$$
\phi (\boldsymbol {x}) = \prod_ {i} \phi_ {i} (\boldsymbol {x} _ {i}).\tag{1.7}
$$

In other words, the independence relationships are encoded by the edges $e _ { i j }$ of the factor graph, with each factor $\phi _ { i }$ a function of only the variables $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { i } }$ in its adjacency set $\mathcal { X } ( \phi _ { i } )$

In the rest of this chapter, we show how to find an optimal assignment, the MAP estimate, through optimization over the unknown variables in the factor graph. Indeed, for an arbitrary factor graph, MAP inference comes down to maximizing the product (1.7) of all factor-graph potentials:

$$
\boldsymbol {x} ^ {\mathrm{MAP}} = \underset {\boldsymbol {x}} {\arg \max} \phi (\boldsymbol {x})\tag{1.8a}
$$

$$
= \underset {\boldsymbol {x}} {\arg \max} \prod_ {i} \phi_ {i} (\boldsymbol {x} _ {i}).\tag{1.8b}
$$

What is left now is to derive the exact form of the factors $\phi _ { i } ( { \pmb x } _ { i } )$ , which depends very much on how we model the measurement models $p ( \boldsymbol { z } | \boldsymbol { x } )$ and the prior densities $p ( { \pmb x } )$ . We discuss this in detail next.

## 1.2.2 Specifying Probability Densities

The exact form of the densities $p ( \boldsymbol { z } | \boldsymbol { x } )$ and $p ( { \pmb x } )$ above depends very much on the application and the sensors used. The most often used densities involve the multivariate Gaussian density, with probability density

$$
\mathcal {N} (\boldsymbol {\theta}; \boldsymbol {\mu}, \boldsymbol {\Sigma}) = \frac {1}{\sqrt {| 2 \pi \boldsymbol {\Sigma} |}} \exp \left(- \frac {1}{2} \| \boldsymbol {\theta} - \boldsymbol {\mu} \| _ {\boldsymbol {\Sigma}} ^ {2}\right),\tag{1.9}
$$

where $\pmb { \mu } \in \mathbb { R } ^ { n }$ is the mean, $\pmb { \Sigma }$ is an $n \times n$ covariance matrix, and

$$
\| \boldsymbol {\theta} - \boldsymbol {\mu} \| _ {\boldsymbol {\Sigma}} ^ {2} \triangleq (\boldsymbol {\theta} - \boldsymbol {\mu}) ^ {\top} \boldsymbol {\Sigma} ^ {- 1} (\boldsymbol {\theta} - \boldsymbol {\mu})\tag{1.10}
$$

denotes the squared Mahalanobis distance. The normalization constant ${ \sqrt { | 2 \pi \Sigma | } } =$ $\left( 2 \pi \right) ^ { n / 2 } \left| \pmb { \Sigma } \right| ^ { 1 / 2 }$ , where $| . |$ denotes the matrix determinant, ensures the multivariate Gaussian density integrates to 1.0 over its domain.

Priors on unknown quantities are often specified using a Gaussian density, and in many cases it is both justified and convenient to model measurements as corrupted by zero-mean Gaussian noise. For example, a bearing measurement<sup>3</sup> from a given pose p to a given landmark ℓ would be modeled as

$$
\boldsymbol {z} = \boldsymbol {h} (\boldsymbol {p}, \ell) + \boldsymbol {\eta},\tag{1.11}
$$

where $h ( \cdot )$ is a measurement prediction function, and the noise η is drawn from a zero-mean Gaussian density with measurement covariance $\Sigma _ { R }$ . This yields the following conditional density $p ( \boldsymbol { z } | \boldsymbol { p } , \boldsymbol { \ell } )$ on the measurement z:

$$
p (\boldsymbol {z} | \boldsymbol {p}, \ell) = \mathcal {N} (\boldsymbol {z}; \boldsymbol {h} (\boldsymbol {p}, \ell), \boldsymbol {\Sigma} _ {\boldsymbol {R}}) = \frac {1}{\sqrt {| 2 \pi \boldsymbol {\Sigma} _ {\boldsymbol {R}} |}} \exp \left(- \frac {1}{2} \| \boldsymbol {z} - \boldsymbol {h} (\boldsymbol {p}, \ell) \| _ {\boldsymbol {\Sigma} _ {\boldsymbol {R}}} ^ {2}\right).\tag{1.12}
$$

The measurement functions $h ( \cdot )$ are often nonlinear in practical robotics applications. Still, while they depend on the sensor used and the SLAM front-end, they are typically not dificult to reason about or write down. The measurement function for a 2D bearing measurement is simply

$$
\pmb {h} (\pmb {p}, \ell) = \mathrm{atan2} (\ell_ {y} - p _ {y}, \ell_ {x} - p _ {x}),\tag{1.13}
$$

where $\ell _ { x } , \ell _ { y }$ are the x and y coordinates of the landmark, $p _ { x } , p _ { y }$ are the x and $_ \mathrm { y }$ coordinates of the pose, and atan2 is the well-known two-argument arctangent variant. Hence, the final probabilistic measurement model $p ( \boldsymbol { z } | \boldsymbol { p } , \boldsymbol { \ell } )$ is obtained as

$$
p (\pmb {z} | \pmb {p}, \pmb {\ell}) = \frac {1}{\sqrt {| 2 \pi \pmb {\Sigma} _ {\pmb {R}} |}} \exp \left(- \frac {1}{2} \| \pmb {z} - \mathrm{atan2} (\ell_ {y} - p _ {y}, \ell_ {x} - p _ {x}) \| _ {\pmb {\Sigma} _ {\pmb {R}}} ^ {2}\right).\tag{1.14}
$$

Note that we will not always assume Gaussian measurement noise: to cope with the occasional data association mistake, for example, many authors have proposed the use of robust measurement densities, with heavier tails than a Gaussian density; these are discussed in Chapter 3.

Not all probability densities involved are derived from measurements. For example, in the toy SLAM problem the prior p(x) on the trajectory is made up of a prior $p ( { \pmb p } _ { 1 } )$ and conditional densities $p ( \pmb { p } _ { t + 1 } | \pmb { p } _ { t } )$ , specifying a probabilistic motion model that the robot is assumed to obey given known control inputs $\mathbf { \pmb { u } } _ { t }$ . In practice, we often use a conditional Gaussian assumption,

$$
p (\pmb {p} _ {t + 1} | \pmb {p} _ {t}, \pmb {u} _ {t}) = \frac {1}{\sqrt {| 2 \pi \pmb {\Sigma} _ {Q} |}} \exp \left(- \frac {1}{2} \| \pmb {p} _ {t + 1} - \pmb {g} (\pmb {p} _ {t}, \pmb {u} _ {t}) \| _ {\pmb {\Sigma} _ {Q}} ^ {2}\right),\tag{1.15}
$$

where $g ( \cdot )$ is a motion model, and $\Sigma _ { Q }$ a covariance matrix of the appropriate dimensionality, $\mathrm { e . g . , 3 \times 3 }$ in the case of robots operating in the plane.

Often we have no known control inputs $\mathbf { \pmb { u } } _ { t }$ but instead we measure how the robot moved, e.g., via an odometry measurement $\mathbf { } _ { o _ { t } }$ . For example, if we assume the odometry simply measures the diference between poses, subject to Gaussian noise with covariance $\Sigma _ { S }$ , we obtain

$$
p (\boldsymbol {o} _ {t} | \boldsymbol {p} _ {t + 1}, \boldsymbol {p} _ {t}) = \frac {1}{\sqrt {| 2 \pi \boldsymbol {\Sigma} _ {S} |}} \exp \left(- \frac {1}{2} \| \boldsymbol {o} _ {t} - (\boldsymbol {p} _ {t + 1} - \boldsymbol {p} _ {t}) \| _ {\boldsymbol {\Sigma} _ {S}} ^ {2}\right).\tag{1.16}
$$

If we have both known control inputs $\mathbf { \pmb { u } } _ { t }$ and odometry measurements $\mathbf { } _ { o _ { t } }$ we can combine (1.15) and (1.16).

Note that for robots operating in three-dimensional space, we will need slightly more sophisticated machinery to specify densities on nonlinear manifolds such as SE(3), as discussed in the next chapter.

## 1.2.3 Nonlinear Least Squares

We now show that MAP inference for SLAM problems with Gaussian noise models as above is equivalent to solving a nonlinear least squares problem. Let us assume that all factors are proportional to a multivariate Gaussian, i.e.,

$$
\phi_ {i} (\boldsymbol {x} _ {i}) \propto \exp \left(- \frac {1}{2} \| \boldsymbol {z} _ {i} - \boldsymbol {h} _ {i} (\boldsymbol {x} _ {i}) \| _ {\boldsymbol {\Sigma} _ {i}} ^ {2}\right).\tag{1.17}
$$

This includes both simple Gaussian priors and likelihood factors derived from measurements corrupted by zero-mean, normally distributed noise. Substituting (1.17) into (1.8b), and then taking the negative log and dropping the factor $\begin{array} { l } { { \frac { 1 } { 2 } } } \end{array}$ allows us to instead minimize a sum of nonlinear least squares terms:

$$
\boldsymbol {x} ^ {\mathrm{MAP}} = \underset {\boldsymbol {x}} {\arg \min} \sum_ {i} \| \boldsymbol {z} _ {i} - \boldsymbol {h} _ {i} (\boldsymbol {x} _ {i}) \| _ {\boldsymbol {\Sigma} _ {i}} ^ {2}.\tag{1.18}
$$

Minimizing this objective function performs sensor fusion through the process of combining several measurement-derived factors, and possibly several priors, to de termine the MAP solution for the unknowns.

An important and non-obvious observation is that the factors in (1.18) typically represent rather under-specified densities on the involved unknown variables $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { i } }$ . Indeed, except for simple prior factors, the measurements $z _ { i }$ are typically of lower dimension than the unknowns $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { i } }$ . In those cases, the factor by itself accords the same likelihood to an infinite subset of the domain of $\mathbf { x } _ { i } .$ . For example, a 2D measurement in a camera image is consistent with an entire ray of 3D points that project to the same image location.

Even though the functions $\boldsymbol { h } _ { i }$ are nonlinear, $i f$ we have a decent initial guess available, then the nonlinear optimization methods we discuss in this chapter will be able to converge to the global minimum of (1.18). We should caution, however, that as our objective in (1.18) is non-convex, there is no guarantee that we will not get stuck in a local minimum if our initial guess is poor. This has led to so-called certifiably optimal solvers, which are the subject of Chapter 6. Below, however, we focus on local methods rather than global solvers. We start of below by considering the easier problem of solving a linearized version of the problem.

## 1.3 Solving Linear Least Squares

Before tackling the more dificult problem of nonlinear least squares, in this section we first show how to linearize the problem, show how this leads to a linear least squares problem, and review matrix factorization as computationally eficient way to solve the corresponding normal equations. A seminal reference for these methods is the book by Golub and Loan [389].

## 1.3.1 Linearization

We can linearize all measurement functions $\pmb { h } _ { i } ( \cdot )$ in the nonlinear least squares objective function (1.18) using a simple Taylor expansion,

$$
\boldsymbol {h} _ {i} (\boldsymbol {x} _ {i}) = \boldsymbol {h} _ {i} (\boldsymbol {x} _ {i} ^ {0} + \boldsymbol {\delta} _ {i}) \approx \boldsymbol {h} _ {i} (\boldsymbol {x} _ {i} ^ {0}) + \boldsymbol {H} _ {i} \boldsymbol {\delta} _ {i},\tag{1.19}
$$

where the measurement Jacobian $\pmb { H } _ { i }$ is defined as the (multivariate) partial derivative of $\pmb { h } _ { i } ( \cdot )$ at a given linearization point $\pmb { x } _ { i } ^ { 0 }$

$$
\boldsymbol {H} _ {i} \triangleq \left. \frac {\partial \boldsymbol {h} _ {i} (\boldsymbol {x} _ {i})}{\partial \boldsymbol {x} _ {i}} \right| _ {\boldsymbol {x} _ {i} ^ {0}},\tag{1.20}
$$

and $\pmb { \delta } _ { i } \triangleq \pmb { x } _ { i } - \pmb { x } _ { i } ^ { 0 }$ is the state update vector. Note that we make an assumption that $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { i } }$ lives in a vector space or, equivalently, can be represented by a vector. This is not always the case, e.g., when some of the unknown states in x represent 3D rotations or other more complex manifold types. We will revisit this issue in Chapter 2.

Substituting the Taylor expansion (1.19) into the nonlinear least squares expression (1.18) we obtain a linear least squares problem in the state update vector $\delta ,$

$$
\boldsymbol {\delta} ^ {*} = \underset {\boldsymbol {\delta}} {\arg \min} \sum_ {i} \left\| \boldsymbol {z} _ {i} - \boldsymbol {h} _ {i} (\boldsymbol {x} _ {i} ^ {0}) - \boldsymbol {H} _ {i} \boldsymbol {\delta} _ {i} \right\| _ {\boldsymbol {\Sigma} _ {i}} ^ {2}\tag{1.21a}
$$

$$
= \arg \min _ {\boldsymbol {\delta}} \sum_ {i} \left\| \left(\boldsymbol {z} _ {i} - \boldsymbol {h} _ {i} (\boldsymbol {x} _ {i} ^ {0})\right) - \boldsymbol {H} _ {i} \boldsymbol {\delta} _ {i} \right\| _ {\boldsymbol {\Sigma} _ {i}} ^ {2},\tag{1.21b}
$$

where ${ z } _ { i } - { h } _ { i } ( { x } _ { i } ^ { 0 } )$ is the prediction error at the linearization point, $i . e .$ , the diference between actual and predicted measurement. Above, $\delta ^ { * }$ denotes the solution to the locally linearized problem.

By a simple change of variables we can drop the covariance matrices $\Sigma _ { i }$ from this point forward: defining $\pmb { \Sigma } ^ { 1 / 2 }$ as the matrix square root of $\Sigma ,$ we can rewrite the square Mahalanobis norm as follows:

$$
\left\| \boldsymbol {e} \right\| _ {\boldsymbol {\Sigma}} ^ {2} \stackrel {{\Delta}} {{=}} \boldsymbol {e} ^ {\top} \boldsymbol {\Sigma} ^ {- 1} \boldsymbol {e} = \left(\boldsymbol {\Sigma} ^ {- 1 / 2} \boldsymbol {e}\right) ^ {\top} \left(\boldsymbol {\Sigma} ^ {- 1 / 2} \boldsymbol {e}\right) = \left\| \boldsymbol {\Sigma} ^ {- 1 / 2} \boldsymbol {e} \right\| _ {2} ^ {2}.\tag{1.22}
$$

Hence, we can eliminate the covariances $\Sigma _ { i }$ by pre-multiplying the Jacobian $\pmb { H } _ { i }$ and the prediction error in each term in (1.21b) with $\pmb { \Sigma } _ { i } ^ { - 1 7 2 }$

$$
\pmb {A} _ {i} = \pmb {\Sigma} _ {i} ^ {- 1 / 2} \pmb {H} _ {i}\tag{1.23a}
$$

$$
\pmb {b} _ {i} = \pmb {\Sigma} _ {i} ^ {- 1 / 2} \left(\pmb {z} _ {i} - \pmb {h} _ {i} (\pmb {x} _ {i} ^ {0})\right).\tag{1.23b}
$$

This process is a form of whitening. For example, in the case of scalar measurements it simply means dividing each term by the measurement standard deviation $\sigma _ { i }$ . Note that this eliminates the units of the measurements $\left( \mathrm { e . g . } \right.$ , length, angles) so that the diferent rows can be combined into a single cost function.

## 1.3.2 SLAM as Linear Least Squares

After linearization, we finally obtain the following standard least squares problem:

$$
\boldsymbol {\delta} ^ {*} = \underset {\boldsymbol {\delta}} {\arg \min} \sum_ {i} \| \boldsymbol {A} _ {i} \boldsymbol {\delta} _ {i} - \boldsymbol {b} _ {i} \| _ {2} ^ {2}\tag{1.24a}
$$

$$
= \underset {\boldsymbol {\delta}} {\arg \min} \| \boldsymbol {A} \boldsymbol {\delta} - \boldsymbol {b} \| _ {2} ^ {2},\tag{1.24b}
$$

Above A and b are obtained by collecting all whitened Jacobian matrices $\mathbf { A } _ { i }$ and whitened prediction errors $\mathbf { } _ { b _ { i } }$ into one large matrix A and right-hand-side (RHS) vector $^ { b , }$ respectively.

The Jacobian A is a large-but-sparse matrix, with a block structure that mirrors the structure of the underlying factor graph. We will examine this sparsity structure in detail below. First, however, we review the classical linear algebra approach to solving this least squares problem.

## 1.3.3 Matrix Factorization for Least Squares

For a full-rank $m \times n$ matrix A, with m $\geq n$ , the unique least squares solution to (1.24b) can be found by solving the normal equations:

$$
\left(\boldsymbol {A} ^ {\top} \boldsymbol {A}\right) \boldsymbol {\delta} ^ {*} = \boldsymbol {A} ^ {\top} \boldsymbol {b}.\tag{1.25}
$$

This is normally done by factoring the information matrix Λ, also called the Hessian matrix, defined and factored as follows:

$$
\boldsymbol {\Lambda} \stackrel {\Delta} {=} \boldsymbol {A} ^ {\top} \boldsymbol {A} = \boldsymbol {R} ^ {\top} \boldsymbol {R}.\tag{1.26}
$$

Above, the Cholesky factor R is an upper-triangular $n \times n \ \mathrm { m a t r i x ^ { 4 } }$ and is computed using Cholesky factorization, a variant of lower-upper (LU) factorization for symmetric positive-definite matrices. After this, $\delta ^ { * }$ can be found by solving first

$$
\boldsymbol {R} ^ {\top} \boldsymbol {y} = \boldsymbol {A} ^ {\top} \boldsymbol {b}\tag{1.27}
$$

for y and then

$$
\boldsymbol {R} \delta^ {*} = \boldsymbol {y}\tag{1.28}
$$

for $\delta ^ { * }$ by forward and backward substitution, respectively. For dense matrices, Cholesky factorization requires $n ^ { 3 } / 3$ flops, and the entire algorithm, including computing half of the symmetric $A ^ { \top } A$ , requires $( m + n / 3 ) n ^ { 2 }$ flops. One could also use lower-diagonal-upper (LDU) factorization, a variant of Cholesky decomposition that avoids the computation of square roots.

An alternative to Cholesky factorization that is more accurate and more numerically stable is to proceed via $\textstyle Q R .$ -factorization, which works without computing the information matrix Λ. Instead, we compute the QR-factorization of A itself along with its corresponding RHS:

$$
\boldsymbol {A} = \boldsymbol {Q} \left[ \begin{array}{c} \boldsymbol {R} \\ \boldsymbol {0} \end{array} \right], \quad \left[ \begin{array}{c} \boldsymbol {d} \\ \boldsymbol {e} \end{array} \right] = \boldsymbol {Q} ^ {\top} \boldsymbol {b}.\tag{1.29}
$$

Here $Q$ is an m m orthogonal matrix, $d \in \mathbb { R } ^ { n } , e \in \mathbb { R } ^ { m - n }$ , and R is the same upper-triangular Cholesky triangle. The preferred method for factorizing a dense matrix A is to compute R column by column, proceeding from left to right. For each column $j ,$ , all nonzero elements below the diagonal are zeroed out by multiplying $\pmb { A }$ on the left with a Householder reflection matrix $H _ { j }$ . After n iterations A is completely factorized:

$$
\boldsymbol {H} _ {n} \dots \boldsymbol {H} _ {2} \boldsymbol {H} _ {1} \boldsymbol {A} = \boldsymbol {Q} ^ {\top} \boldsymbol {A} = \left[ \begin{array}{c} \boldsymbol {R} \\ \boldsymbol {0} \end{array} \right].\tag{1.30}
$$

The orthogonal matrix $Q$ is not usually formed: instead, the transformed RHS $\pmb { Q } ^ { \top } \pmb { b }$ is computed by appending $^ { b }$ as an extra column to A. Because the $Q$ factor is orthogonal, we have

$$
\left\| \boldsymbol {A} \delta - \boldsymbol {b} \right\| _ {2} ^ {2} = \left\| \boldsymbol {Q} ^ {\top} \boldsymbol {A} \delta - \boldsymbol {Q} ^ {\top} \boldsymbol {b} \right\| _ {2} ^ {2} = \left\| \boldsymbol {R} \delta - \boldsymbol {d} \right\| _ {2} ^ {2} + \left\| \boldsymbol {e} \right\| _ {2} ^ {2},\tag{1.31}
$$

where we made use of the equalities from (1.29). Clearly, $\left\| e \right\| _ { 2 } ^ { 2 }$ will be the least squares sum of squared residuals, and the least squares solution $\delta ^ { * }$ can be obtained by solving the triangular system

$$
\boldsymbol {R} \delta^ {*} = \boldsymbol {d}\tag{1.32}
$$

via back-substitution. Note that the upper-triangular factor R obtained using QR factorization is the same (up to possible sign changes on the diagonal) as would be obtained by Cholesky factorization, since

$$
\boldsymbol {A} ^ {\top} \boldsymbol {A} = \left[ \begin{array}{c} \boldsymbol {R} \\ \boldsymbol {0} \end{array} \right] ^ {\top} \boldsymbol {Q} ^ {\top} \boldsymbol {Q} \left[ \begin{array}{c} \boldsymbol {R} \\ \boldsymbol {0} \end{array} \right] = \boldsymbol {R} ^ {\top} \boldsymbol {R},\tag{1.33}
$$

where we again made use of the fact that $Q$ is orthogonal. The cost of QR is dominated by the cost of the Householder reflections, which is $2 ( m - n / 3 ) n ^ { 2 }$ . Comparing this with Cholesky, we see that both algorithms require $O ( m n ^ { 2 } )$ operations when m n, but that QR-factorization is slower by a factor of 2.

In summary, the linearized optimization problem associated with SLAM can be concisely stated in terms of basic linear algebra. It comes down to factorizing either the information matrix Λ or the measurement Jacobian A into square-root form. Because they are based on matrix square roots derived from the SAM problem, we have referred to this family of approaches as square-root SAM, or SAM for short [249, 254].

## 1.4 Nonlinear Optimization

In this section, we discuss some classic optimization approaches to the nonlinear least squares problem defined in (1.18). As a reminder, in SLAM the nonlinear least squares objective function is given by

$$
J (\boldsymbol {x}) \triangleq \sum_ {i} \| \boldsymbol {z} _ {i} - \boldsymbol {h} _ {i} (\boldsymbol {x} _ {i}) \| _ {\boldsymbol {\Sigma} _ {i}} ^ {2}\tag{1.34}
$$

and corresponds to a nonlinear factor graph derived from the measurements along with prior densities on some or all unknowns.

Nonlinear least squares problems cannot be solved directly in general, but require an iterative solution starting from a suitable initial estimate. Nonlinear optimization methods do so by solving a succession of linear approximations to (1.18) in order to approach the minimum [264]. A variety of algorithms exist that difer in how they locally approximate the cost function, and in how they find an improved estimate based on that local approximation. A general in-depth treatment of nonlinear solvers is provided by [814], while [389] focuses on the linear-algebra perspective.

All of the algorithms share the following basic structure: they start from an initial estimate $\mathbf { \boldsymbol { x } } ^ { 0 }$ . In each iteration, an update step δ is calculated and applied to obtain the next estimate $\pmb { x } ^ { t + 1 } = \pmb { x } ^ { t } { + } \pmb { \delta }$ . This process ends when certain convergence criteria are reached, such as the norm of the change δ falling below a small threshold.

## 1.4.1 Steepest Descent

Steepest Descent (SD) uses the direction of steepest descent at the current estimate to calculate the following update step,

$$
\pmb {\delta} ^ {\mathrm{sd}} = - \alpha \left. \nabla J (\pmb {x}) \right| _ {\pmb {x} = \pmb {x} ^ {t}},\tag{1.35}
$$

where $\mathbf { \boldsymbol { x } } ^ { t }$ is the current estimate for x. Here the negative gradient is used to identify the direction of steepest descent. For the nonlinear least squares objective function (1.34), we locally approximate the objective function as a quadratic, $J ( \pmb { x } ) \approx \left\| \pmb { A } ( \pmb { x } - \pmb { x } ^ { t } ) - \pmb { b } \right\| _ { 2 } ^ { 2 } ,$ , and obtain the exact gradient $\nabla J \left( \pmb { x } \right) | _ { \pmb { x } = \pmb { x } ^ { t } } = - 2 \pmb { A } ^ { \top } i$ b at the linearization point $\mathbf { \boldsymbol { x } } ^ { t }$

The step size α needs to be carefully chosen to balance between safe updates and reasonable convergence speed. An explicit line search can be performed to find a minimum in the given direction. SD is a simple algorithm, but sufers from slow convergence near the minimum.

## 1.4.2 Gauss-Newton

Gauss-Newton (GN) provides faster convergence by using a second-order update. GN exploits the special structure of the nonlinear least squares problem to approximate the Hessian using the Jacobian as $A ^ { \top } A$ . The GN update step is obtained by

solving the normal equations (1.25),

$$
\boldsymbol {A} ^ {\top} \boldsymbol {A} \delta^ {\mathrm{gn}} = \boldsymbol {A} ^ {\top} \boldsymbol {b},\tag{1.36}
$$

by any of the methods in Section 1.3.3. For a well-behaved (i.e., nearly quadratic) objective function and a good initial estimate, Gauss-Newton exhibits nearly quadratic convergence. If the quadratic fit is poor, a GN step can lead to a new estimate that is further from the minimum and subsequent divergence.

## 1.4.3 Levenberg-Marquardt

The Levenberg-Marquardt (LM) algorithm allows for iterating multiple times to convergence while controlling in which region one is willing to trust the quadratic approximation made by Gauss-Newton. Hence, such a method is often called a trust-region method.

To combine the advantages of both the SD and GN methods, Levenberg [646] proposed to modify the normal equations (1.25) by adding a non-negative constant $\lambda \in \mathbb { R } ^ { + } \cup \lbrace 0 \rbrace$ to the diagonal

$$
\left(\boldsymbol {A} ^ {\top} \boldsymbol {A} + \lambda \mathbf {I}\right) \boldsymbol {\delta} ^ {\mathrm{lm}} = \boldsymbol {A} ^ {\top} \boldsymbol {b}.\tag{1.37}
$$

Note that for $\lambda = 0$ we obtain GN, and for large λ we approximately obtain $\delta ^ { * }$ ≈ ${ \frac { 1 } { \lambda } } A ^ { \top } b$ , an update in the negative gradient direction of the cost function J (1.34). Hence, LM can be seen to blend naturally between the GN and SD methods.

Marquardt [731] later proposed to take into account the scaling of the diagonal entries to provide faster convergence:

$$
\left(\boldsymbol {A} ^ {\top} \boldsymbol {A} + \lambda \operatorname{diag} (\boldsymbol {A} ^ {\top} \boldsymbol {A})\right) \boldsymbol {\delta} ^ {\mathrm{lm}} = \boldsymbol {A} ^ {\top} \boldsymbol {b}.\tag{1.38}
$$

This modification causes larger steps in the steepest-descent direction if the gradient is small (nearly flat directions of the objective function), because there the inverse of the diagonal entries will be large. Conversely, in steep directions of the objective function the algorithm becomes more cautious and takes smaller steps. Both modifications of the normal equations can be interpreted in Bayesian terms as adding a zero-mean prior on all unknown variables in the problem.

A key diference between GN and LM is that the latter rejects updates that would lead to a higher sum of squared residuals. A rejected update means that the nonlinear function is locally not well-behaved, and smaller steps are needed. This is achieved by heuristically increasing the value of $\lambda ,$ for example by multiplying its current value by a factor of 10, and resolving the modified normal equations. On the other hand, if a step leads to a reduction of the sum of squared residuals, it is accepted, and the state estimate is updated accordingly. In this case, λ is reduced (e.g., by dividing by a factor of 10), and the algorithm repeats with a new linearization point, until convergence.

![](images/63399f14c6144bf40f6d42b715dddac9f3d787a13de54bbcd30bc393236308ae.jpg)  
Figure 1.6 Powell’s dogleg algorithm combines the separately computed Gauss-Newton and gradient descent update steps.

## 1.4.4 Dogleg Minimization

Powell’s dogleg (PDL) algorithm [885] can be a more eficient alternative to LM [697]. A major disadvantage of the Levenberg-Marquardt algorithm is that in case a step gets rejected, the modified information matrix has to be refactored, which is the most expensive component of the algorithm. Hence, the key idea behind PDL is to separately compute the GN and SD steps, and then combine appropriately. If the LM step gets rejected, the directions of the GN and SD steps are still valid, and they can be combined in a diferent way until a reduction in the cost is achieved. Hence, each update of the state estimate only involves one matrix factorization, as opposed to several.

Figure 1.6 shows how the GN and SD steps are combined. The combined step starts with the SD update, followed by a sharp bend (hence the term dogleg) towards the GN update, but stopping at the trust region boundary. Unlike LM, PDL maintains an explicit trust region within which we trust the linear assumption. The appropriateness of the linear approximation is determined by the gain ratio

$$
\rho = \frac {J (\boldsymbol {x} ^ {t}) - J (\boldsymbol {x} ^ {t} + \boldsymbol {\delta})}{L (\boldsymbol {0}) - L (\boldsymbol {\delta})},\tag{1.39}
$$

where $\boldsymbol { L } ( \delta ) = \boldsymbol { A } ^ { \intercal } \boldsymbol { A } \delta - \boldsymbol { A } ^ { \intercal } \boldsymbol { b }$ is the linearization of the nonlinear quadratic cost function J from (1.34) at the current estimate $\mathbf { \boldsymbol { x } } ^ { t }$ . If $\rho$ is small $( \mathrm { i . e . , ~ } \rho ~ < ~ 0 . 2 5 )$ then the cost has not reduced as predicted by the linearization and the trust region is reduced. On the other hand, if the reduction is as predicted (or better, i.e., $\rho > 0 . 7 5 )$ , then the trust region is increased depending on the magnitude of the update vector, and the step is accepted.

## 1.5 Factor Graphs and Sparsity

The solvers presented so far assume that the matrices involved may be dense. Dense methods will not scale to realistic problem sizes in SLAM. For the toy problem in

Figure 1.1 a dense method will work fine. The larger simulation example, with its factor graph shown in Figure 1.5, is more representative of real-world problems. However, it is still relatively small as real SLAM problems go, where problems with thousands or even millions of unknowns are common. Yet, we are able to handle these without a problem because of sparsity.

The sparsity can be appreciated directly from looking at the factor graph. It is clear from Figure 1.5 that the graph is sparse (i.e., it is by no means a fully connected graph). The odometry chain linking the 100 unknown poses is a linear structure of 100 binary factors, instead of the possible $1 0 0 ^ { 2 }$ (binary) factors. In addition, with 20 landmarks we could have up to 2000 factors linking each landmark to each pose: the true number is closer to 400. And finally, there are no factors between landmarks at all. This reflects that we have not been given any information about their relative position. This structure is typical of most SLAM problems.

## 1.5.1 The Sparse Jacobian and its Factor Graph

The key to modern SLAM algorithms is exploiting sparsity, and an important property of factor graphs in SLAM is that they represent the sparse block structure in the resulting sparse Jacobian matrix A. To see this, let us revisit the least squares problem that is the key computation in the inner loop of the nonlinear SLAM problem:

$$
\boldsymbol {\delta} ^ {*} = \underset {\boldsymbol {\delta}} {\arg \min} \sum_ {i} \| \boldsymbol {A} _ {i}   \boldsymbol {\delta} _ {i} - \boldsymbol {b} _ {i} \| _ {2} ^ {2}.\tag{1.40}
$$

Each term above is derived from a factor in the original, nonlinear SLAM problem, linearized around the current linearization point (1.21b). The matrices $\mathbf { A } _ { i }$ can be broken up in blocks corresponding to each variable, and collected in a large, blocksparse Jacobian whose sparsity structure is given exactly by the factor graph.

Even though these linear problems typically arise as inner iterations in nonlinear optimization, we drop the δ notation below, as everything holds for general linear problems regardless of their origin.

Consider the factor graph for the small toy example in Figure 1.1. After linearization, we obtain a sparse system [A b] with the block structure in Figure 1.7. Comparing this with the factor graph, it is obvious that every factor corresponds to a block-row, and every variable corresponds to a block-column of A. In total there are nine block-rows, one for every factor in the factorization of $\phi ( p _ { 1 } , p _ { 2 } , p _ { 3 } , \ell _ { 1 } , \ell _ { 2 } )$ .

## 1.5.2 The Sparse Information Matrix and its Graph

When using Cholesky factorization for solving the normal equations, as explained in Section 1.3.3, we first form the Hessian or information matrix $\pmb { \Lambda } = \pmb { A } ^ { \top } \pmb { A }$ . Note that $A ^ { \top } A$ is not the true Hessian but the “Gauss-Newton” approximation, obtained

$$
[ \boldsymbol {A} | \boldsymbol {b} ] = \begin{array}{c c} & \delta \ell_ {1} \quad \delta \ell_ {2} \quad \delta \boldsymbol {p} _ {1} \quad \delta \boldsymbol {p} _ {2} \quad \delta \boldsymbol {p} _ {3} \quad \boldsymbol {b} \\ \phi_ {1} & \\ \phi_ {2} & \\ \phi_ {3} & \\ \phi_ {4} & \\ \phi_ {5} & \\ \phi_ {6} & \\ \phi_ {7} & \\ \phi_ {8} & \\ \phi_ {9} & \end{array} \left[ \begin{array}{c c c c c c} & & \boldsymbol {A} _ {1 3} & & & \boldsymbol {b} _ {1} \\ & & \boldsymbol {A} _ {2 3} & \boldsymbol {A} _ {2 4} & & \boldsymbol {b} _ {2} \\ & & & \boldsymbol {A} _ {3 4} & \boldsymbol {A} _ {3 5} & \boldsymbol {b} _ {3} \\ \boldsymbol {A} _ {4 1} & & & & & \boldsymbol {b} _ {4} \\ & \boldsymbol {A} _ {5 2} & & & & \boldsymbol {b} _ {5} \\ & & \boldsymbol {A} _ {6 3} & & & \boldsymbol {b} _ {6} \\ \boldsymbol {A} _ {7 1} & & \boldsymbol {A} _ {7 3} & & & \boldsymbol {b} _ {7} \\ \boldsymbol {A} _ {8 1} & & & \boldsymbol {A} _ {8 4} & & \boldsymbol {b} _ {8} \\ & \boldsymbol {A} _ {9 2} & & & \boldsymbol {A} _ {9 5} & \boldsymbol {b} _ {9} \end{array} \right]
$$

Figure 1.7 Block structure of the sparse Jacobian A for the toy SLAM example in Figure 1.1 with $\pmb { \delta } = \left[ \delta \pmb { \ell } _ { 1 } ^ { \top } \ \delta \pmb { \ell } _ { 2 } ^ { \top } \right.$ δp<sup>⊤</sup><sub>1</sub> $\delta \pmb { p } _ { 2 } ^ { \top } \delta \bar { \pmb { p } _ { 3 } ^ { \top } } ] ^ { \top }$ . Blank entries are zeros.  
![](images/9e5631051f6c2aeb5026d609b884773bd2902fc60e87816bfa9f5ecda4ab10d4.jpg)

![](images/4f020457cc25171d369116da03c553cf2f3bd9c2efab51f4b769b89ac0e14913.jpg)  
(a) Information matrix $\pmb { \Delta } \triangleq \pmb { A } ^ { \top } \pmb { A } .$  
(b) Undirected graph associated with Λ.  
Figure 1.8 Information matrix Λ and its associated undirected graph G for the toy SLAM problem.

by truncating a Taylor series of the residual. In general, since the Jacobian A is block-sparse, the Hessian Λ is also expected to be sparse. By construction, Λ is symmetric, and if a unique MAP solution exists, it is positive definite.

The sparsity pattern of Λ naturally defines an undirected graph G, where an edge is present between two variables if they ever co-occur in the same factor. At the block level, the sparsity pattern of $\pmb { \Lambda } = \pmb { A } ^ { \top } \pmb { A }$ is exactly the adjacency matrix of this graph. This generalizes beyond pairwise factors: an n-ary factor induces a clique among all its variables, and hence nonzero blocks among all pairs.

Figure 1.8 shows (a) the block structure of the information matrix Λ for our running toy example, and (b) the corresponding undirected graph G. We will often refer to this graph G associated with Λ to reason about sparsity and fill-in. <sup>5</sup>

## 1.5.3 Sparse Factorization

We have seen MAP estimation amounts to solving a linear system of equations as described in Section 1.3.3. In the case of nonlinear least squares problems, we solve such a system repeatedly in an iterative setup. We have seen in the previous two sections that both A and $A ^ { \top } A$ enjoy sparsity determined by the factor graph. Without going into detail, this known sparsity pattern can be used to greatly speed up either Cholesky factorization (in the case of working with $A ^ { \top } A )$ or QRfactorization (in the case of working with A). Eficient software implementations are available, e.g., CHOLMOD [189] and SuiteSparseQR [240], which are also used under the hood by several software packages. In practice, sparse Cholesky or LDU factorization outperform QR factorization on sparse problems as well, and not just by a constant factor.

The flop count for sparse factorization will be much lower than for a dense matrix. Crucially, the column ordering chosen for the sparse matrices can dramatically influence the total flop-count. While any order will ultimately produce an identical MAP estimate, the variable order determines the fill-in of matrix factors (i.e., the extra nonzero entries beyond the sparsity pattern of the matrix being factored). It is known that finding the variable ordering that minimizes fill-in during matrix factorization is an NP-hard problem [1239], so we must resort to using good heuristics. This will in turn afect the computational complexity of the factorization algorithm.

We demonstrate this by way of an example. Recall the larger simulation example, with its factor graph shown in Figure 1.5. The sparsity patterns for the corresponding sparse Jacobian matrix A is shown in Figure 1.9. Also shown is the pattern for the information matrix $\pmb { \Lambda } \triangleq \pmb { A } ^ { \top } \pmb { A }$ , in the top-right corner. On the right of Figure 1.9, we show the resulting upper triangular Cholesky factor R for two diferent orderings. Both of them are sparse, and both of them satisfy $\pmb { R } ^ { \top } \pmb { R } = \pmb { A } ^ { \top } \pmb { A }$ (up to a permutation of the variables), but they difer in the amount of sparsity they exhibit. It is exactly this that will determine how expensive it is to factorize A. The first version of the ordering comes naturally: the poses come first and then the landmarks, leading to a sparse R factor with 9399 nonzero entries. In contrast, the sparse factor R in the bottom right was obtained by reordering the variables according to the Column approximate minimum degree permutation (COLAMD) heuristic [29, 241] and only has 4168 nonzero entries. Yet back-substitution gives exactly the same solution for both versions.

It is worth mentioning that other tools, like pre-conditioned conjugate gradient, can solve the normal equations iteratively. In visual SLAM, which has a very specific sparsity pattern, power iterations have also been used successfully [1171]. However, sparse factorization is still the method of choice for most SLAM problems and has a nice graphical model interpretation, which we discuss next.

![](images/7e17c938008bfc073e129913c90a743cd7814473ae879a14b1a0258fe94435c9.jpg)

![](images/c1adcb707df7de40435c8961f2d8d90e8a153cb2331c986cdc6c9a529f47237e.jpg)

![](images/e27122ad220643ced8681777660a4d77364b7933619024206ae497e9c5f20901.jpg)

![](images/f5fad5e1ec472a6b428b7a6fe55523c80ff02dd7cd4a0aea9f1935d0fb1fde57.jpg)  
Figure 1.9 On the left, the measurement Jacobian A associated with the problem in Figure 1.5, which has $3 \times 9 5 + 2 \times 2 4 = 3 3 3$ unknowns. The number of rows, 1126, is equal to the number of (scalar) measurements. Also given is the number of nonzero entries $" \mathrm { n n z } ^ { \prime \prime }$ . On the right: (top) the information matrix $\mathbf { \Delta } \Lambda \triangleq A ^ { \intercal } A ;$ (middle) its upper triangular Cholesky triangle R; (bottom) an alternative factor amdR obtained with a better variable ordering (COLAMD).

## 1.6 Elimination

We have so far restricted ourselves to a linear-algebra explanation of performing inference for SLAM. In this section, we expand our worldview by thinking about inference more abstractly using graphical models directly. This will ultimately lead us to current state-of-the-art SLAM solvers based on a concept called the Bayes tree for incremental smoothing and mapping in the next section.

## 1.6.1 Variable Elimination Algorithm

There exists a general algorithm that can, given any (preferably sparse) factor graph, compute the corresponding posterior density $p ( { \pmb x } | z )$ on the unknown variables x in a form that allows easy recovery of the MAP solution to the problem. As we saw, a factor graph represents the unnormalized posterior $\phi ( \pmb { x } ) \propto p ( \pmb { x } | \pmb { z } )$ as a product of factors, and in SLAM problems this graph is typically generated directly from the measurements. The variable elimination algorithm is a recipe for converting a factor graph into another graphical model called a Bayes net, which depends only on the unknown variables x. This then allows for easy MAP inference (as well as other operations such as sampling and/or marginalization).

In particular, the variable elimination algorithm is a way to factorize any factor graph of the form

$$
\phi (\boldsymbol {x}) = \phi (\boldsymbol {x} _ {1}, \dots , \boldsymbol {x} _ {n})\tag{1.41}
$$

into a factored Bayesian network probability density of the form

$$
p (\boldsymbol {x}) = p \left(\boldsymbol {x} _ {1} \mid \boldsymbol {s} _ {1}\right) p \left(\boldsymbol {x} _ {2} \mid \boldsymbol {s} _ {2}\right) \dots p \left(\boldsymbol {x} _ {n}\right) = \prod_ {j} p \left(\boldsymbol {x} _ {j} \mid \boldsymbol {s} _ {j}\right),\tag{1.42}
$$

where $s _ { j }$ denotes an assignment to the separator $\pmb { s } ( \pmb { x } _ { j } )$ associated with variable $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { j } }$ under the chosen variable ordering $\pmb { x } _ { 1 } , \ldots , \pmb { x } _ { n }$ . The separator is defined as the set of variables on which $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { j } }$ is conditioned, after elimination. While this factorization is akin to the chain rule, eliminating a sparse factor graph will typically lead to small separators.

The elimination algorithm proceeds by eliminating one variable $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { \mathcal { I } } }$ at a time, starting with the complete factor graph $\phi _ { 1 : n }$ . As we eliminate each variable $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { j } }$ we generate a single conditional $p ( \pmb { x } _ { j } | \pmb { s } _ { j } )$ , as well as a reduced factor graph $\phi _ { j + 1 }$ :n on the remaining variables. After all variables have been eliminated, the algorithm returns the resulting Bayes net with the desired factorization.

To eliminate a single variable $\boldsymbol { \mathscr { x } } _ { j }$ given a partially eliminated factor graph $\phi _ { j : n } ,$ we first remove all factors $\phi _ { i } ( { \pmb x } _ { i } )$ that are adjacent to $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { j } }$ and multiply them into the product factor $\psi ( \pmb { x } _ { j } , \pmb { s } _ { j } )$ . We then factorize $\psi ( \pmb { x } _ { j } , \pmb { s } _ { j } )$ into a conditional distribution $p ( \pmb { x } _ { j } | \pmb { s } _ { j } )$ on the eliminated variable $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { j } }$ , and a new factor $\tau ( s _ { j } )$ ) on the separator s :

$$
\psi (\boldsymbol {x} _ {j}, \boldsymbol {s} _ {j}) = p (\boldsymbol {x} _ {j} | \boldsymbol {s} _ {j}) \tau (\boldsymbol {s} _ {j}).\tag{1.43}
$$

![](images/f367ec43cda5f3fdc37108cc58b8d68b91a0b1c5fd955c49cbdb6bffb688dce4.jpg)  
Figure 1.10 Variable elimination for the toy SLAM example, transforming the factor graph from Figure 1.3 into a Bayes net (bottom right), using the ordering $\ell _ { 1 } , \ell _ { 2 } , p _ { 1 } , p _ { 2 } , p _ { 3 } .$

Hence, the entire factorization from $\phi ( { \pmb x } )$ to $p ( { \pmb x } )$ is seen to be a succession of n local factorization steps. When eliminating the last variable ${ \pmb x } _ { n }$ the separator $s _ { n }$ will be empty, and the conditional produced will simply be a prior $p ( { \pmb x } _ { n } )$ on ${ \mathbf { \mathcal { x } } } _ { n }$

One possible elimination sequence for the toy example is shown in Figure 1.10, for the ordering $\ell _ { 1 } , \ell _ { 2 } , p _ { 1 } , p _ { 2 } , p _ { 3 }$ . In each step, the variable being eliminated is shaded gray, and the new factor $\tau ( s _ { j } )$ on the separator $s _ { j }$ is shown in red. Taken as a whole, the variable elimination algorithm factorizes the factor graph $\phi ( \ell _ { 1 } , \ell _ { 2 } , p _ { 1 } , p _ { 2 } , p _ { 3 } )$ into the Bayes net in Figure 1.10 (bottom right), corresponding to the factorization

$$
p \left(\ell_ {1}, \ell_ {2}, \boldsymbol {p} _ {1}, \boldsymbol {p} _ {2}, \boldsymbol {p} _ {3}\right) = p \left(\ell_ {1} \mid \boldsymbol {p} _ {1}, \boldsymbol {p} _ {2}\right) p \left(\ell_ {2} \mid \boldsymbol {p} _ {3}\right) p \left(\boldsymbol {p} _ {1} \mid \boldsymbol {p} _ {2}\right) p \left(\boldsymbol {p} _ {2} \mid \boldsymbol {p} _ {3}\right) p \left(\boldsymbol {p} _ {3}\right).\tag{1.44}
$$

## 1.6.2 Linear-Gaussian Elimination

In the case of linear measurement functions and additive normally distributed noise, the elimination algorithm is equivalent to sparse matrix factorization. Both sparse Cholesky and QR factorization are a special case of the general algorithm.

As explained before, the elimination algorithm proceeds one variable at a time. For every variable $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { j } }$ we remove all factors $\phi _ { i } ( { \pmb x } _ { i } )$ adjacent to $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { j } }$ and form the intermediate product factor $\psi ( \pmb { x } _ { j } , \pmb { s } _ { j } )$ . This can be done by accumulating all the matrices $\mathbf { A } _ { i }$ into a new, larger block-matrix ${ \bar { A } } _ { j } ,$ as we can write

$$
\psi (\boldsymbol {x} _ {j}, \boldsymbol {s} _ {j}) \leftarrow \prod_ {i \in \mathcal {N} _ {j}} \phi_ {i} (\boldsymbol {x} _ {i})\tag{1.45a}
$$

$$
= \exp \left(- \frac {1}{2} \sum_ {i} \| \boldsymbol {A} _ {i} \boldsymbol {x} _ {i} - \boldsymbol {b} _ {i} \| _ {2} ^ {2}\right)\tag{1.45b}
$$

$$
= \exp \left(- \frac {1}{2} \left\| \bar {\boldsymbol {A}} _ {j} [ \boldsymbol {x} _ {j}; \boldsymbol {s} _ {j} ] - \bar {\boldsymbol {b}} _ {j} \right\| _ {2} ^ {2}\right),\tag{1.45c}
$$

where the new RHS vector ${ \bar { \boldsymbol { b } } } _ { j }$ stacks all $\mathbf { } _ { b _ { i } }$ and $\stackrel { \cdot , \cdot } { \mathrm { ~ , ~ } }$ also denotes vertical stacking.

Consider eliminating the variable $\ell _ { 1 }$ in the toy example. The adjacent factors are $\phi _ { 4 } , \phi _ { 7 }$ , and $\phi _ { 8 }$ , in turn inducing the separator $\pmb { s } _ { 1 } = [ \pmb { p } _ { 1 } ; \pmb { p } _ { 2 } ]$ . The product factor is then equal to

$$
\psi \left(\boldsymbol {\ell} _ {1}, \boldsymbol {p} _ {1}, \boldsymbol {p} _ {2}\right) = \exp \left(- \frac {1}{2} \left\| \bar {\boldsymbol {A}} _ {1} [ \boldsymbol {\ell} _ {1}; \boldsymbol {p} _ {1}; \boldsymbol {p} _ {2} ] - \bar {\boldsymbol {b}} _ {1} \right\| _ {2} ^ {2}\right),\tag{1.46}
$$

with

$$
\bar {\boldsymbol {A}} _ {1} \triangleq \left[ \begin{array}{c c c} \boldsymbol {A} _ {4 1} & & \\ \boldsymbol {A} _ {7 1} & \boldsymbol {A} _ {7 3} & \\ \boldsymbol {A} _ {8 1} & & \boldsymbol {A} _ {8 4} \end{array} \right], \qquad \bar {\boldsymbol {b}} _ {1} \triangleq \left[ \begin{array}{c} \boldsymbol {b} _ {4} \\ \boldsymbol {b} _ {7} \\ \boldsymbol {b} _ {8} \end{array} \right].\tag{1.47}
$$

Looking at the sparse Jacobian in Figure 1.7, this simply boils down to taking out the block-rows with nonzero blocks in the first column, corresponding to the three factors adjacent to $\ell _ { 1 }$ .

Next, factorizing the product $\psi ( \pmb { x } _ { j } , \pmb { s } _ { j } )$ can be done in several diferent ways. We discuss the QR variant, as it more directly connects to the linearized factors. In particular, the augmented matrix $[ { \bar { A _ { j } } } | { \bar { b } } _ { j } ]$ corresponding to the product factor $\psi ( \pmb { x } _ { j } , \pmb { s } _ { j } )$ can be rewritten using partial QR-factorization [389] as follows:

$$
[ \bar {\boldsymbol {A}} _ {j} | \bar {\boldsymbol {b}} _ {j} ] = \boldsymbol {Q} \left[ \begin{array}{c c c} \boldsymbol {R} _ {j} & \boldsymbol {T} _ {j} & \boldsymbol {d} _ {j} \\ & \tilde {\boldsymbol {A}} _ {\tau} & \tilde {\boldsymbol {b}} _ {\tau} \end{array} \right],\tag{1.48}
$$

where $R _ { j }$ is an upper-triangular matrix. This allows us to factor $\psi ( \pmb { x } _ { j } , \pmb { s } _ { j } )$ as follows:

![](images/7f7da02ffac9b2ac7ca9019d72b500da0d139492bdb2841af98e305211d9b176.jpg)  
Figure 1.11 Eliminating the variable $\ell _ { 1 }$ as a partial sparse factorization step.

$$
\begin{array}{l} \psi (\boldsymbol {x} _ {j}, \boldsymbol {s} _ {j}) = \exp \left\{- \frac {1}{2} \left\| \bar {\boldsymbol {A}} _ {j} [ \boldsymbol {x} _ {j}; \boldsymbol {s} _ {j} ] - \bar {\boldsymbol {b}} _ {j} \right\| _ {2} ^ {2} \right\} \\ = \exp \left\{- \frac {1}{2} \left\| \boldsymbol {R} _ {j} \boldsymbol {x} _ {j} + \boldsymbol {T} _ {j} \boldsymbol {s} _ {j} - \boldsymbol {d} _ {j} \right\| _ {2} ^ {2} \right\} \exp \left\{- \frac {1}{2} \left\| \tilde {\boldsymbol {A}} _ {\tau} \boldsymbol {s} _ {j} - \tilde {\boldsymbol {b}} _ {\tau} \right\| _ {2} ^ {2} \right\} \\ = p (\boldsymbol {x} _ {j} | \boldsymbol {s} _ {j}) \tau (\boldsymbol {s} _ {j}), \end{array}\tag{1.49a}
$$

(1.49b)

where we used the fact that the rotation matrix Q does not alter the value of the norms involved.

In the toy example, Figure 1.11 shows the result of eliminating the first variable in the example, the landmark $\ell _ { 1 }$ with separator $[ p _ { 1 } ; p _ { 2 } ]$ . We show the operation on the factor graph and the corresponding efect on the sparse Jacobian from Figure 1.7, omitting the RHS. The partition above the line corresponds to a sparse, upper-triangular matrix R that is being formed. New contributions to the matrix are shown: blue for the contributions to R, and red for newly created factors. For completeness, we show the four remaining variable elimination steps in Figure 1.12, showing an end-to-end example of how QR factorization proceeds on a small example. The final step shows the equivalence between the resulting Bayes net and the sparse upper-triangular factor R.

The entire elimination algorithm, using partial QR to eliminate a single variable, is equivalent to sparse QR factorization. As the treatment above considers multi-dimensional variables $\pmb { x } _ { j } \in \mathbb { R } ^ { n _ { j } }$ , this is in fact an instance of multi-frontal QR factorization [288], as we eliminate several scalar variables at a time, which is beneficial for processor utilization. While in our case the scalar variables are grouped because of their semantic meaning in the inference problem, sparse linear algebra codes typically analyze the problem to group for maximum computational eficiency. In many cases these two strategies are closely aligned.

![](images/e2e8d95287a0da35908e51410277a99b2c009af7c8e501191f52e42f89f6f4b6.jpg)  
Figure 1.12 The remaining elimination steps for the toy example, completing a full QR factorization. The last step in the bottom right shows the equivalence between the resulting Bayes net and the sparse Cholesky factor R.

## 1.6.3 Sparse Cholesky Factor as a Bayes Net

The equivalence between variable elimination and sparse matrix factorization reveals that the graphical model associated with an upper triangular matrix is a Bayes net! Just like a factor graph is the graphical embodiment of a sparse Jacobian, a Bayes net reveals the sparsity structure of a Cholesky factor. In hindsight, this perhaps is not too surprising: a Bayes net is a directed acyclic graph (DAG), and that is exactly the ‘upper-triangular’ property for matrices.

What’s more, the Cholesky factor corresponds to a Gaussian Bayes net, which we define as one made up of linear-Gaussian conditionals. The variable elimination algorithm holds for general densities, but in case the factor graph only contains linear measurement functions and Gaussian additive noise, the resulting Bayes net has a very specific form. We discuss the details below, as well as how to solve for the MAP estimate in the linear case.

As we discussed, the Gaussian factor graph corresponding to the linearized nonlinear problem is transformed by elimination into the density $p ( { \pmb x } )$ given by the now-familiar Bayes-net factorization:

$$
p (\boldsymbol {x}) = \prod_ {j} p (\boldsymbol {x} _ {j} | \boldsymbol {s} _ {j}).\tag{1.50}
$$

In both QR and Cholesky variants, the conditional densities $p ( \pmb { x } _ { j } | \pmb { s } _ { j } )$ are given by

$$
p (\boldsymbol {x} _ {j} | \boldsymbol {s} _ {j}) = k \exp \left(- \frac {1}{2} \| \boldsymbol {R} _ {j} \boldsymbol {x} _ {j} + \boldsymbol {T} _ {j} \boldsymbol {s} _ {j} - \boldsymbol {d} _ {j} \| _ {2} ^ {2}\right),\tag{1.51}
$$

which is a linear-Gaussian density on the eliminated variable $\mathbf { \boldsymbol { x } } _ { j }$ . Indeed, we have

$$
\| \boldsymbol {R} _ {j} \boldsymbol {x} _ {j} + \boldsymbol {T} _ {j} \boldsymbol {s} _ {j} - \boldsymbol {d} _ {j} \| _ {2} ^ {2} = (\boldsymbol {x} _ {j} - \boldsymbol {\mu} _ {j}) ^ {\top} \boldsymbol {R} _ {j} ^ {\top} \boldsymbol {R} _ {j} (\boldsymbol {x} _ {j} - \boldsymbol {\mu} _ {j}) \stackrel {{\Delta}} {{=}} \| \boldsymbol {x} _ {j} - \boldsymbol {\mu} _ {j} \| _ {\Sigma_ {j}} ^ {2},\tag{1.52}
$$

where the mean $\pmb { \mu } _ { j } = \pmb { R } _ { j } ^ { - 1 } ( \pmb { d } _ { j } - \pmb { T } _ { j } \pmb { s } _ { j } )$ depends linearly on the separator $s _ { j }$ , and the covariance matrix is given by $\pmb { \Sigma } _ { j } = ( \pmb { R } _ { j } ^ { \top } \pmb { R } _ { j } ) ^ { - 1 }$ . Hence, the normalization constant 7 $k = | 2 \pi \Sigma _ { j } | ^ { - \frac { 1 } { 2 } }$

After the elimination step is complete, back-substitution is used to obtain the MAP estimate of each variable. As seen in Figure 1.12, the last variable eliminated does not depend on any other variables. Thus, the MAP estimate of the last variable can be directly extracted from the Bayes net. By proceeding in reverse elimination order, the values of all the separator variables for each conditional will always be available from the previous steps, allowing the estimate for the current frontal variable to be computed.

At every step, the MAP estimate for the variable $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { j } }$ is the conditional mean,

$$
\pmb {x} _ {j} ^ {*} = \pmb {R} _ {j} ^ {- 1} (\pmb {d} _ {j} - \pmb {T} _ {j} \pmb {s} _ {j} ^ {*}),\tag{1.53}
$$

since by construction the MAP estimate for the separator $s _ { j } ^ { * }$ is fully known by this point.

## 1.7 Incremental SLAM

In an incremental SLAM setting, we want to compute the optimal trajectory and map whenever we receive new measurements while traversing the environment, or at least at regular intervals. One way to do so is to update the most recent matrix factorization with the new measurements, to reuse the computation that already incorporated all previous measurements. In the linear case, this is possible through incremental factorization methods, the dense versions of which are also discussed at length in Golub and Loan [389]. However, matrix factorization operates on linear systems, but most SLAM problems of practical interest are nonlinear. Using incremental matrix factorization, it is far from obvious how re-linearization can be performed incrementally without refactoring the complete matrix. To overcome this problem we once again resort to graphical models, and introduce a new graphical model, the Bayes tree. We then show how to incrementally update the Bayes tree as new measurements and states are added to the system, leading to the incremental smoothing and mapping (iSAM) algorithm.

## 1.7.1 The Bayes Tree

It is well known that inference in a tree-structured graph is eficient. In contrast, the factor graphs associated with typical robotics problems contain many loops. Still, we can construct a tree-structured graphical model in a two-step process: first, perform variable elimination on the factor graph to obtain a Bayes net with a special property. Second, exploit that special property to find a tree structure over cliques in this Bayes net.

In particular, a Bayes net obtained by running the elimination algorithm on a factor graph satisfies a special property: it is chordal, meaning that any undirected cycle of length greater than three has a chord, i.e., an edge connecting two nonconsecutive vertices on the cycle. In AI and machine learning, a chordal graph is more commonly said to be triangulated. Because it is still a Bayes net, the corresponding joint density $p ( { \pmb x } )$ is given by factorizing over the individual variables $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { j } }$

$$
p (\boldsymbol {x}) = \prod_ {j} p (\boldsymbol {x} _ {j} | \boldsymbol {\pi} _ {j}),\tag{1.54}
$$

where $\pi _ { j }$ are the parent nodes of $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { j } }$ . However, although the Bayes net is chordal, at this variable level it is still a non-trivial graph: neither chain-like nor tree-structured. The chordal Bayes net for our running toy SLAM example is shown in the last step of Figure 1.10, and it is clear that there is an undirected cycle ${ \pmb p } _ { 1 } - { \pmb p } _ { 2 } - { \pmb \ell } _ { 1 }$ , implying it does not have a tree-structured form.

![](images/fbbc1d3911ca63b0e0846df9a3c898cb9f9f8f87cfeeb727c0c02f0bc1503bce.jpg)  
(a)

![](images/2f6dfc1527081b319d833490f6b19bed5384b31b0b0a98873e06b02f749005ac.jpg)

![](images/985d80a75c18ed847d85ce95b26652216276bc2461141486e09f9b73be82ce83.jpg)  
(c)  
Figure 1.13 The Bayes tree (b) and the associated square root information matrix R (c) describing the clique structure in the chordal Bayes net (a) based on our canonical example from Figure 1.3. A Bayes tree is similar to a clique tree, but is better at capturing the formal equivalence between sparse linear algebra and inference in graphical models. The association of cliques with rows in the R factor is indicated by color.

By identifying cliques in this chordal graph, the Bayes net may be rewritten as a Bayes tree. We introduce this new, tree-structured graphical model to capture the clique structure of the Bayes net. It is not obvious that cliques in the Bayes net should form a tree. They do so because of the chordal property, although we will not attempt to prove that here. Listing all these cliques in an undirected tree yields a clique tree, also known as a junction tree in AI and machine learning. The Bayes tree is just a directed version of this that preserves information about the elimination order.

More formally, a Bayes tree is a directed tree where the nodes represent cliques $c _ { k }$ of the underlying chordal Bayes net. In particular, we define one conditional density $p ( f _ { k } | s _ { k } )$ per node, with the separator $\scriptstyle { \pmb { s } } _ { k }$ as the intersection $\mathbf { c } _ { k } \cap \varpi _ { k }$ of the clique $c _ { k }$ and its parent clique $\varpi _ { k }$ . The frontal variables $f _ { k }$ are the remaining variables, i.e., $\pmb { f } _ { k } \triangleq \pmb { c } _ { k } \setminus \pmb { s } _ { k }$ . Notationally, we write $\pmb { c } _ { k } = \pmb { f } _ { k } : \pmb { s } _ { k }$ for a clique. The following expression gives the joint density $p ( { \pmb x } )$ on the variables x defined by a Bayes tree:

$$
p (\boldsymbol {x}) = \prod_ {k} p (\boldsymbol {f} _ {k} | \boldsymbol {s} _ {k}).\tag{1.55}
$$

For the root $f _ { r } ,$ the separator is empty, i.e., it is a simple prior $p ( f _ { r } )$ on the root variables. The way Bayes trees are defined, the separator $\scriptstyle { \pmb { s } } _ { k }$ for a clique $c _ { k }$ is always a subset of the parent clique $\varpi _ { k }$ , and hence the directed edges in the graph have the same semantic meaning as in a Bayes net: conditioning.

The Bayes tree associated with our canonical toy SLAM problem (Figure 1.3) is shown in Figure 1.13. The root clique ${ \pmb { c } } _ { 1 } = { \pmb { p } } _ { 2 } , { \pmb { p } } _ { 3 }$ (shown in blue) comprises p and $\mathbf { \mathit { p } } _ { 3 }$ , which intersects with two other cliques, $\pmb { c } _ { 2 } = \pmb { \ell } _ { 1 } , \pmb { p } _ { 1 } : \pmb { p } _ { 2 }$ shown in green, and $c _ { 3 } = \ell _ { 2 } : p _ { 3 }$ shown in red. The colors also indicate how the rows of squareroot information matrix R map to the diferent cliques, and how the Bayes tree captures independence relationships between them. For example, the green and red rows only intersect in variables that belong to the root clique, as predicted.

## 1.7.2 Updating the Bayes Tree

Incremental inference corresponds to a simple editing of the Bayes tree. This view provides a better explanation and understanding of the otherwise abstract incremental matrix factorization process. It also allows us to store and compute the square-root information matrix in the form of a Bayes tree, a deeply meaningful sparse storage scheme.

To incrementally update the Bayes tree, we selectively convert part of the Bayes tree back into factor-graph form. When a new measurement is added this corresponds to adding a factor, e.g., a measurement involving two variables will induce a new binary factor $\phi ( \pmb { x } _ { j } , \pmb { x } _ { j ^ { \prime } } )$ . In this case, only the paths in the Bayes tree between the cliques containing $\boldsymbol { \mathscr { x } } _ { j }$ and $\mathbf { \Delta } _ { \mathbf { x } _ { j ^ { \prime } } }$ and the root will be afected. The sub-trees below these cliques are unafected, as are any other sub-trees not containing $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { j } }$ or $\mathbf { \Delta } _ { \mathbf { x } _ { j ^ { \prime } } }$ Hence, to update the Bayes tree, the afected parts of the tree are converted back into a factor graph, and the new factor associated with the new measurement is added to it. By re-eliminating this temporary factor graph, using whatever elimination ordering is convenient, a new Bayes tree is formed and the unafected sub-trees can be reattached.

In order to understand why only the top part of the tree is afected, we look at two important properties of the Bayes tree. These directly arise from the fact that it encodes the information flow during elimination. The Bayes tree is formed from the chordal Bayes net following the inverse elimination order. In this way, variables in each clique collect information from their child cliques via the elimination of these children. Thus, information in any clique propagates only upwards to the root. Second, the information from a factor enters elimination only when the first variable connected to that factor is eliminated. Combining these two properties, we see that a new factor cannot influence any other variables that are not successors of the factor’s variables. However, a factor involving variables having diferent $( { \mathrm { i . e . } }$ independent) paths to the root means that these paths must now be re-eliminated to express the new dependency between them.

![](images/98b3df5e9a1923be27d3989025df8e6389c8ec7789e6940c5665c3f4fc5b28a2.jpg)  
Figure 1.14 Updating a Bayes tree with a new factor, based on the example in Figure 1.13. The afected part of the Bayes tree is highlighted for the case of adding a new factor between $\pmb { p } _ { 1 }$ and $\pmb { p } _ { 3 }$ . Note that the right branch (green) is not afected by the change. (top right) The factor graph generated from the afected part of the tree with the new factor (dashed blue) inserted. (bottom right) The chordal Bayes net resulting from eliminating the factor graph. (bottom left) The Bayes tree created from the chordal Bayes net, with the unmodified right ‘orphan’ sub-tree from the original Bayes tree added back in.

Figure 1.14 shows how these incremental factorization/inference steps are applied to our canonical SLAM example. In this example, we add a new factor between ${ \pmb p } _ { 1 }$ and $\mathbf { \mathit { p } } _ { 3 }$ , afecting only the left branch of the tree, marked by the red dashed line in to top-left figure. We then create the factor graph shown in the top-right figure by creating a factor for each of the clique densities, $p ( p _ { 2 } , p _ { 3 } )$ and $p ( \ell _ { 1 } , p _ { 1 } | p _ { 2 } )$ 2 and add the new factor $f ( p _ { 1 } , p _ { 3 } )$ . The bottom-right figure shows the eliminated graph using the ordering $\ell _ { 1 } , p _ { 1 } , p _ { 2 } , p _ { 3 }$ . And finally, in the bottom-left figure, the reassembled Bayes tree is shown consisting of two parts: the Bayes tree derived from the eliminated graph, and the unafected clique from the original Bayes tree (shown in green).

Figure 1.15 shows an example of the Bayes tree for a small SLAM sequence. Shown is the tree for step 400 of the well-known Manhattan world simulated sequence by Olson et al. [824]. As a robot explores the environment, new measurements only afect parts of the tree, and only those parts are re-calculated.

![](images/db0dc3716404698bb8196e20dffe797c7631e8ef07a6701cc33568c83b3aeaf9.jpg)  
Figure 1.15 An example of the Bayes tree data structure for a small SLAM sequence. The incremental nonlinear least squares estimation algorithm iSAM2 [534] is based on viewing incremental factorization as editing the graphical model corresponding to the posterior probability of the solution, the Bayes tree. As a robot explores the environment, new measurements often only afect small parts of the tree, and only those parts are recalculated (shown in red).

## 1.7.3 Incremental Smoothing and Mapping

Putting all of the above together and addressing some practical considerations about re-linearization yields a state-of-the-art incremental, nonlinear approach to MAP estimation in robotics, iSAM. The first version, iSAM1[531], used the incremental matrix factorization methods from Golub and Loan [389]. However, linearization in iSAM1 was handled in a sub-optimal way: it was done for the full factor graph at periodic instances and/or when matrix fill-in became unwieldy. The second version of the approach, iSAM2, uses a Bayes tree representation for the posterior density [534]. It then employs Bayes tree incremental updating as each new measurement comes in, as described above.

What variable ordering should we use in re-eliminating the afected cliques? Only the variables in the afected part of the Bayes tree are updated. One strategy then is to apply COLAMD locally to the afected variables. However, we can do better: we force recently accessed variables to the end of the ordering, i.e., into the root clique. For this incremental variable ordering strategy one can use the constrained COLAMD algorithm [241]. This both forces the most recently accessed variables to the end and still provides a good overall ordering. Generally, subsequent updates will then only afect a small part of the tree, and can therefore be expected to be eficient in most cases, except for large loop closures.

After updating the tree we also need to update the solution. Back-substitution in the Bayes tree proceeds from the root (which does not depend on any other variables) and proceeds to the leaves. However, it is typically not necessary to recompute a solution for all variables: local updates to the tree often do not afect variables in remote parts of the tree. Instead, at each clique we can check the diference in variable estimates that is propagated downwards and stop when this diference falls below a small threshold.

Our motivation for introducing the Bayes tree was to incrementally solve nonlinear optimization problems. For this we selectively re-linearize factors that contain variables whose deviation from the linearization point exceeds a small threshold. In contrast to the tree modification above, we now have to redo all cliques that contain the afected variables, not just as frontal variables, but also as separator variables. This afects larger parts of the tree, but in most cases is still significantly cheaper than recomputing the complete tree. We also have to go back to the original factors, instead of directly turning the cliques into a factor graph. And that requires caching certain quantities during elimination. The overall incremental nonlinear algorithm, iSAM2, is described in much more detail in [534].

iSAM1 and iSAM2 have been applied successfully to many diferent robotics estimation problems with non-trivial constraints between variables that number into the millions, as will be discussed subsequent chapters. Both are implemented in the GTSAM library, which can be found at https://github.com/borglab/gtsam.

## 1.8 Further Readings & Recent Trends

For those readers interested in learning more details about factor graphs, we recommend the longer article by Dellaert et al. [255]. Out of necessity, this chapter is somewhat brief and we have downplayed the more advanced concepts so as to keep the barrier to entry as low as possible. However, factor graphs and the general elimination algorithm are quite powerful and worth knowing in some detail. After this, we recommend looking at Kaess et al. [534] to better understand the concept of the Bayes tree, which underlies modern solvers such as GTSAM.

Today, the factor graph paradigm can be used to handle state variables on manifolds, incorporate many diferent kinds of sensors, made to handle outlier measurements, and can even fold in the outputs of deep learning methods. Much of the rest of this handbook discusses these trends and so to learn more, keep reading.

Advanced State Variable Representations

Timothy Barfoot, Frank Dellaert, Michael Kaess, and Jose Luis Blanco-Claraco

The previous chapter detailed how to set up and solve a SLAM problem using the factor-graph paradigm. We deliberately avoided discussing some subtleties of the state variables we were estimating. In this chapter, we revisit the nature of our state variables and introduce two important topics that are prevalent in modern SLAM formulations. First and foremost, we need some better tools for handling state variables that have certain constraints associated with them; these constraints define a manifold for our variables, which subsequently then require special care during optimization. There are many examples of manifolds that appear in SLAM, the most common being those associated with the rotational aspects of a robot (especially in three dimensions, but even in the plane). A second aspect of state variables stems from the nature of time itself. In the previous chapter, we implicitly assumed that our robot moved in discrete-time steps through the world. In this chapter we introduce smooth, continuous-time representations of trajectories and discuss how these are fully compatible with our factor-graph formulation. We use Barfoot [47] as the primary reference with some streamlined notation from Sola et al. [1022].

## 2.1 Optimization on Manifolds

While in some robotics problems we can get away with vector-valued unknowns, in most practical situations we have to deal with three-dimensional rotations and other non-vector manifolds. Loosely speaking, a manifold is a topologically closed surface $( e . g .$ , the perimeter of a circle, or the surface of a sphere); importantly, a manifold resembles Euclidean space locally near each point. Manifolds require a more sophisticated machinery that takes into account their special structure. In this section, we discuss how to perform optimization on manifolds, which will build upon the optimization framework for vector spaces from the previous chapter. As an example, Figure 2.1 visualizes a spherical manifold, , and its tangent space, $T _ { x } { \mathcal { M } }$ , which can be used as a local coordinate system at $x \in \mathcal { M }$ for optimization.

![](images/bbe04e3dd68402335deae02f97a01ba9bd5413b1b433cd0d96e4948fd713088a.jpg)  
Figure 2.1 For the sphere manifold, M, the local tangent plane, $T _ { x } { \mathcal { M } } .$ , with a local basis provides the notion of local coordinates.

## 2.1.1 Rotations and Poses

While there are several manifolds that can be discussed in the context of SLAM, the two most common are those used to represent rotations and poses. Rotations are typically either in two (planar) or three dimensions and we therefore refer to the manifold of rotations as the special orthogonal group $\mathrm { S O } ( d )$ , where $d = 2$ or 3, accordingly. A planar rotation matrix, $R _ { a } ^ { b } \in \mathrm { S O } ( 2 )$ , has the form

$$
\boldsymbol {R} _ {a} ^ {b} = \left[ \begin{array}{c c} \cos \theta & - \sin \theta \\ \sin \theta & \cos \theta \end{array} \right],\tag{2.1}
$$

where $\theta \in \mathbb { R }$ , the angle of rotation, is the single degree of freedom in this case. Moreover, $R _ { a } ^ { b }$ allows us to rotate a two-dimensional vector $( i . e .$ , landmark) expressed in reference frame ${ \mathcal { F } } ^ { a }$ to $\mathcal { F } ^ { b } \colon \ell ^ { b } = R _ { a } ^ { b } \ell ^ { a }$

A rotation matrix in three dimensions, $R _ { a } ^ { b } \in \mathrm { S O } ( 3 )$ , again rotates vectors (this time in three dimensions) from one frame to another. Three-dimensional rotation matrices have nine entries but only three degrees of freedom $( e . g .$ , roll, pitch, yaw). Both two- and three-dimensional rotation matrices must satisfy the constraints $\pmb { R } _ { a } ^ { b \top } \pmb { R } _ { a } ^ { b } = \mathbf { I }$ and det $( \boldsymbol { R _ { a } ^ { b } } ) = 1$ to limit their degrees of freedom appropriately.

The pose of a robot comprises both rotational, $R _ { a } ^ { b } \in \mathrm { S O } ( d )$ , and translational, $t _ { a } ^ { b } \in \mathbb { R } ^ { d }$ , variables with $3 ( d - 1 )$ ) degrees of freedom in all. Sometimes we keep track of these quantities separately and then can use $\{ R _ { a } ^ { b } , \pmb { t } _ { a } ^ { b } \} \in \mathrm { S O } ( d ) \times \mathbb { R } ^ { d }$ as the representation. Alternatively, these quantities can be assembled into a $( d { + } 1 ) { \times } ( d { + } 1 )$ transformation matrix,

$$
\boldsymbol {T} _ {a} ^ {b} = \left[ \begin{array}{c c} \boldsymbol {R} _ {a} ^ {b} & \boldsymbol {t} _ {a} ^ {b} \\ \boldsymbol {0} & 1 \end{array} \right].\tag{2.2}
$$

The manifold of all such transformation matrices is called the special Euclidean group, $\operatorname { S E } ( d )$ , where again $d = 2$ (planar motion) or 3 (three-dimensional motion). The benefit of using $\operatorname { S E } ( d )$ is that we can easily translate and rotate landmarks

using a single matrix multiplication:

$$
\underbrace {\left[ \begin{array}{c} \boldsymbol {\ell} ^ {b} \\ 1 \end{array} \right]} _ {\tilde {\boldsymbol {\ell}} ^ {b}} = \underbrace {\left[ \begin{array}{c c} \boldsymbol {R} _ {a} ^ {b} & \boldsymbol {t} _ {a} ^ {b} \\ \boldsymbol {0} & 1 \end{array} \right]} _ {\boldsymbol {T} _ {a} ^ {b}} \underbrace {\left[ \begin{array}{c} \boldsymbol {\ell} ^ {a} \\ 1 \end{array} \right]} _ {\tilde {\boldsymbol {\ell}} ^ {a}},\tag{2.3}
$$

where $\tilde { \ell }$ is the homogeneous representation of the landmark ℓ.

Due to the constraints imposed on the forms of rotation and transformation matrices, they are unfortunately not vectors. For example, we cannot simply add two rotation matrices together and arrive at another valid rotation matrix. However, it turns out that $\mathrm { S O } ( d )$ and $\operatorname { S E } ( d )$ are examples of manifolds that possess some extra useful properties, called matrix Lie groups. We can exploit the structure of these manifolds to continue to perform unconstrained MAP optimization for factor-graph SLAM (see, for example, Dellaert et al. [255], Boumal [106], or Barfoot [47]). For additional background on Lie groups in robotics see the seminal work of Chirikjian and Kyatkin [207], Chirikjian [205, 206].

## 2.1.2 Matrix Lie Groups

The key to performing optimization on $\mathrm { S O } ( d )$ and $\operatorname { S E } ( d )$ is to exploit their group structure. For example, one nice property is that matrix Lie groups enjoy closure under matrix multiplication, so that if we multiply two members, $e . g . , R _ { b } ^ { c } , R _ { a } ^ { b } \in$ $\mathrm { S O } ( d )$ , the result is also in the group: ${ \pmb R } _ { a } ^ { c } = { \pmb R } _ { b } ^ { c } { \pmb R } _ { a } ^ { b } \in \mathrm { S O } ( d )$

Another nice property of matrix Lie groups is that they come along with a very useful companion structure called a Lie algebra, which is also the tangent space for the Lie group at the identity element. For our purposes, the most important aspects of the Lie algebra are (i) that it comprises a vector space with dimension equal to the number of degrees of freedom of its Lie group, and (ii) there is a well-established mapping (the matrix exponential) from the Lie algebra to the Lie group. This allows us to construct elements of the Lie group with relative ease from elements of the Lie algebra. For example, for SO(2) we can build a rotation matrix (dropping super/subscripts for now) according to

$$
\boldsymbol {R} = \operatorname{Exp} (\theta) = \exp (\theta^ {\wedge}) = \sum_ {n = 0} ^ {\infty} \frac {1}{n !} (\theta^ {\wedge}) ^ {n} = \left[ \begin{array}{c c} \cos \theta & - \sin \theta \\ \sin \theta & \cos \theta \end{array} \right] \in \operatorname{SO} (2),\tag{2.4}
$$

where

$$
\theta^ {\wedge} = \left[ \begin{array}{c c} 0 & - \theta \\ \theta & 0 \end{array} \right],   \theta \in \mathbb {R}.\tag{2.5}
$$

As summarized in Figure 2.2 along with the rest of the operators introduced in this chapter, the quantity $\theta ^ { \wedge }$ is a member of the Lie algebra, $\mathrm { s o } ( 2 )$ , and it is mapped through the matrix exponential, exp( ), to a member of the Lie group, R. We can $_ \mathrm { g o }$ the other way with the matrix logarithm: $\theta = \operatorname { L o g } ( R ) = \left( \log ( R ) \right) ^ { \vee }$ , where we use the matrix logarithm and the vee operator, $( \cdot ) ^ { \vee }$ , which is the inverse of the wedge operator, $( \cdot ) ^ { \wedge }$

![](images/804afd145bba6076b0a4f9b2a7a50f021aa403f602569619e3b9cc5ac1ed7830.jpg)  
Figure 2.2 Summary of notation introduced in this chapter: the wedge $( \cdot ) ^ { \wedge }$ and vee $( \cdot ) ^ { \vee }$ operators, the exponential exp(·) and logarithm $\log ( \cdot )$ maps, which often are just the matrix exponential and logarithm, and the convenience shortcut operators $\mathrm { E x p } ( \cdot ) = \exp ( ( \cdot ) ^ { \wedge } )$ and $\mathrm { L o g } ( \cdot ) = ( \log ( \cdot ) ) ^ { \vee }$ . The $\operatorname { S O } ( n )$ manifold is used for illustrative purposes but operators are defined for any other manifold.

Each matrix Lie group has its own linear $( \cdot ) ^ { \wedge }$ operator used to construct a Lie algebra member from the standard vector space of appropriate dimension. For $\mathrm { S O ( 3 ) }$ 1 it is the skew-symmetric operator:

$$
\boldsymbol {R} = \operatorname{Exp} (\boldsymbol {\theta}) = \exp (\boldsymbol {\theta} ^ {\wedge}) = \sum_ {n = 0} ^ {\infty} \frac {1}{n !} \boldsymbol {\theta} ^ {\wedge^ {n}} \in \mathrm{SO} (3),\tag{2.6a}
$$

$$
\boldsymbol {\theta} ^ {\wedge} = \left[ \begin{array}{c c c} 0 & - \theta_ {3} & \theta_ {2} \\ \theta_ {3} & 0 & - \theta_ {1} \\ - \theta_ {2} & \theta_ {1} & 0 \end{array} \right] \in \operatorname{so} (3), \quad \boldsymbol {\theta} = \left[ \begin{array}{c c c} 0 & - \theta_ {3} & \theta_ {2} \\ \theta_ {3} & 0 & - \theta_ {1} \\ - \theta_ {2} & \theta_ {1} & 0 \end{array} \right] ^ {\vee} = \left[ \begin{array}{c} \theta_ {1} \\ \theta_ {2} \\ \theta_ {3} \end{array} \right] \in \mathbb {R} ^ {3}.\tag{2.6b}
$$

For $\operatorname { S E } ( d )$ , we can use

$$
\boldsymbol {T} = \mathrm{Exp} (\boldsymbol {\xi}) = \mathrm{exp} (\boldsymbol {\xi} ^ {\wedge}) \in \mathrm{SE} (d),\tag{2.7a}
$$

$$
\boldsymbol {\xi} ^ {\wedge} = \left[ \begin{array}{c c} \boldsymbol {\theta} ^ {\wedge} & \boldsymbol {\rho} \\ \mathbf {0} & 0 \end{array} \right] \in \operatorname{se} (d), \quad \boldsymbol {\xi} = \left[ \begin{array}{c} \boldsymbol {\rho} \\ \boldsymbol {\theta} \end{array} \right] \in \mathbb {R} ^ {3 (d - 1)}, \quad \boldsymbol {\theta} \in \mathbb {R} ^ {2 d - 3}, \quad \boldsymbol {\rho} \in \mathbb {R} ^ {d},\tag{2.7b}
$$

where $d = 2$ (planar) or 3 (three-dimensional). Note, the version of the $( \cdot ) ^ { \wedge }$ operator can be determined by the size of the input vector.

As we saw for $\mathrm { S O } ( 2 )$ above, for each of the matrix Lie groups discussed here, there are also well-known closed-form expressions for the mappings between the Lie algebra and the Lie group that can be used rather than the infinite series form of the matrix exponential [47].

## 2.1.3 Lie Group Optimization

Now that we have these matrix Lie groups established, we can use them to help ‘linearize’ our nonlinear least-squares terms in order to carry out MAP inference. Looking back to the discussion in Section 1.3.1, we still seek to linearize our measurement functions, $\pmb { h } _ { i } ( \cdot )$ , only now the input to these may involve a member of a Lie group.

For example, suppose $\pmb { h } _ { i } ( \cdot )$ represents a camera model that takes as its input a homogeneous landmark expressed in the camera frame, $\tilde { \ell } _ { i } ^ { c }$ , and returns the pixel coordinates of the landmark in an image, $z _ { i } \in \mathbb { R } ^ { 2 } \colon z _ { i } = h _ { i } ( \tilde { \ell } _ { i } ^ { c } )$ ). We can write the generative sensor model therefore as

$$
\boldsymbol {z} _ {i} = \boldsymbol {h} _ {i} \left(\boldsymbol {T} _ {w} ^ {c} \tilde {\ell} _ {i} ^ {w}\right) + \boldsymbol {\eta} _ {i},\tag{2.8}
$$

where $\pmb { T } _ { w } ^ { c } \in \mathrm { S E } ( 3 )$ is the pose of world frame with respect to the camera, $\tilde { \ell } _ { i } ^ { w }$ is the homogeneous landmark expressed in the world frame, and $\eta _ { i }$ is the usual sensor noise. We then might like to solve the optimization problem

$$
\boldsymbol {T} _ {w} ^ {c ^ {*}} = \underset {\boldsymbol {T} _ {w} ^ {c}} {\arg \min} = \sum_ {i} \left\| \boldsymbol {z} _ {i} - \boldsymbol {h} _ {i} \left(\boldsymbol {T} _ {w} ^ {c} \tilde {\ell} _ {i} ^ {w}\right) \right\| _ {\boldsymbol {\Sigma} _ {i}} ^ {2},\tag{2.9}
$$

which is known as the perspective-n-point (PNP) problem. For this example, we assume that the positions of the landmarks in the world frame are known but of course in SLAM we might like to estimate these as well.

To linearize our sensor model, we use the fact that we can produce a perturbed version of our pose through its Lie algebra according $\mathrm { t o } ^ { 1 }$

$$
\pmb {T} _ {w} ^ {c} = \pmb {T} _ {w} ^ {c ^ {0}} \mathrm{Exp} (\pmb {\xi}).\tag{2.10}
$$

Here, $\pmb { \xi } \in \mathbb { R } ^ { 6 }$ is used to produce a ‘small’ pose change that perturbs an initial guess, ${ \pmb T } _ { w } ^ { c ^ { 0 } } \in \mathrm { S E } ( 3 )$ . This perturbation is also sometimes written succinctly using the operator so that

$$
\pmb {T} _ {w} ^ {c} = \pmb {T} _ {w} ^ {c ^ {0}} \oplus \pmb {\xi}\tag{2.11}
$$

is a shorthand for (2.10). Owing to the closure property discussed earlier, the product of these two quantities is guaranteed to be in SE(3). By using the Lie algebra to define our pose perturbation, we restrict its dimension to be equal to the actual number of degrees of freedom in a three-dimensional pose, which will mean that we can avoid introducing constraints during optimization.

We can also approximate the perturbed pose according to

$$
\boldsymbol {T} _ {w} ^ {c} \approx \boldsymbol {T} _ {w} ^ {c ^ {0}} \left(\mathbf {I} + \boldsymbol {\xi} ^ {\wedge}\right),\tag{2.12}
$$

where we have kept just the terms up to linear in $\boldsymbol { \xi }$ from the series form of the matrix exponential; see (2.6). Then, inserting (2.12) into our measurement function (2.8), we have

$$
\boldsymbol {z} _ {i} \approx \boldsymbol {h} _ {i} \left(\boldsymbol {T} _ {w} ^ {c ^ {0}} \left(\mathbf {I} + \boldsymbol {\xi} ^ {\wedge}\right) \tilde {\ell} _ {i} ^ {w}\right) + \boldsymbol {\eta} _ {i}.\tag{2.13}
$$

This can also be rewritten as

$$
\boldsymbol {z} _ {i} \approx \boldsymbol {h} _ {i} \left(\boldsymbol {T} _ {w} ^ {c ^ {0}} \tilde {\ell} _ {i} ^ {w} + \boldsymbol {T} _ {w} ^ {c ^ {0}} \tilde {\ell} _ {i} ^ {w ^ {\odot}} \boldsymbol {\xi}\right) + \boldsymbol {\eta} _ {i},\tag{2.14}
$$

where $\odot$ is a (linear) operator for homogeneous points [47]:

$$
\tilde {\boldsymbol {\ell}} ^ {\odot} = \left[ \begin{array}{c} \boldsymbol {\ell} \\ 1 \end{array} \right] ^ {\odot} = \left[ \begin{array}{c c} \mathbf {I} & - \boldsymbol {\ell} ^ {\wedge} \\ \mathbf {0} & \mathbf {0} \end{array} \right].\tag{2.15}
$$

We have essentially ‘linearized’ the pose perturbation in (2.14) and now need to linearize the camera function $\pmb { h } _ { i } ( \cdot )$ as well. We can use a standard first-order Taylor series approximation to write

$$
\boldsymbol {z} _ {i} \approx \boldsymbol {h} _ {i} \left(\boldsymbol {T} _ {w} ^ {c ^ {0}} \tilde {\ell} _ {i} ^ {w}\right) + \underbrace {\left. \frac {\partial \boldsymbol {h} _ {i}}{\partial \boldsymbol {\ell}} \right| _ {\boldsymbol {T} _ {w} ^ {c ^ {0}} \tilde {\ell} _ {i} ^ {w}} \boldsymbol {T} _ {w} ^ {c ^ {0}} \tilde {\ell} _ {i} ^ {w ^ {\odot}}} _ {\boldsymbol {H} _ {i} (\text {chain rule})} \boldsymbol {\xi} + \boldsymbol {\eta} _ {i},\tag{2.16}
$$

where the chaining of two pieces into the overall Jacobian, $\begin{array} { r } { H _ { i } = \frac { \partial h _ { i } } { \partial \xi } } \end{array}$ , is now clear.

An alternative derivation for the same Jacobian can be reached by following a step-by-step decomposition of the sensor model into elementary operations (sensor model, pose composition, exponential operator, etc.) using stacked versions of matrices as intermediary vector states, as described in Blanco [87]. For this particular example, the Jacobian of the camera model with respect to the camera pose can be split into the product of four terms using the chain rule:

$$
\begin{array}{l} \boldsymbol {H} _ {i} = \frac {\partial \boldsymbol {h} _ {i}}{\partial \boldsymbol {\xi}} = \\ \underbrace {\left. \frac {\partial \boldsymbol {h} (\tilde {\ell})}{\partial \tilde {\ell}} \right| _ {\tilde {\ell} = \boldsymbol {T} _ {w} ^ {c ^ {0}} \tilde {\ell} _ {i} ^ {w}}} _ {\text { sensor   model }} \cdot \underbrace \left. \frac {\partial \boldsymbol {T} \tilde {\ell}}{\partial \boldsymbol {T}} \right| _ {\boldsymbol {T} = \boldsymbol {T} _ {w} ^ {c ^ {0}}} \cdot \underbrace {\left. \frac {\partial \boldsymbol {T} _ {1} \oplus \boldsymbol {T} _ {2}}{\partial \boldsymbol {T} _ {2}} \right| _ {\boldsymbol {T} _ {1} = \boldsymbol {T} _ {w} ^ {c ^ {0}}} \cdot \underbrace {\left. \frac {\partial \mathrm{Exp} (\boldsymbol {\xi})}{\partial \boldsymbol {\xi}} \right| _ {\boldsymbol {\xi} = 0}} _ {\text { two   pose   composition }}} _ {\text { exponential   map }}, \end{array} \tag {2.17}
$$

where each individual Jacobian has a known expression [87]. Note how the linearization point for each Jacobian takes into account that the perturbation happens around ${ \pmb \xi } = { \bf 0 }$

Looking back to (1.21b), we can write the linearized least-squares term (i.e., negative-log factor) for this measurement as

$$
\left\| \left(\boldsymbol {z} _ {i} - \boldsymbol {h} _ {i} \left(\boldsymbol {T} _ {w} ^ {c ^ {0}} \tilde {\ell} _ {i} ^ {w}\right)\right) - \boldsymbol {H} _ {i} \boldsymbol {\xi} \right\| _ {\boldsymbol {\Sigma} _ {i}} ^ {2},\tag{2.18}
$$

where the only unknown is our pose perturbation, ξ. After combining this with other factors and then solving for the optimal updates to our state variables, including $\xi ^ { * }$ , we need to update our initial guess, $\pmb { T } _ { w } ^ { c ^ { 0 } }$ . For this, we must return to the perturbation scheme we chose in (2.10) and update according to

$$
\boldsymbol {T} _ {w} ^ {c ^ {0}} \leftarrow \boldsymbol {T} _ {w} ^ {c ^ {0}} \oplus \boldsymbol {\xi} ^ {*},\tag{2.19}
$$

to ensure our solution, $\pmb { T _ { w } ^ { c ^ { 0 } } }$ , remains in SE(3). As usual, optimization proceeds iteratively until the change in all the state variable updates (including $\pmb { \xi } )$ is suficiently small. Note that (2.19) applies when using pose perturbations on the right-hand side as done in (2.10), but this was a choice and in some cases perturbing on the left may be preferable – see Barfoot [47] for more details on this issue.

Often we will encounter measurements that also live on manifolds. For example, we might receive a noisy pose measurement, $\boldsymbol { Z } _ { i }$ , of $\pmb { T } _ { w } ^ { c } \in \mathrm { S E } ( 3 )$ . In this case, we formulate the error in the Lie algebra so that

$$
\left\| \mathrm{Log} \left(\pmb {T} _ {w} ^ {c} \pmb {Z} _ {i} ^ {- 1}\right) \right\| _ {\pmb {\Sigma} _ {i}} ^ {2} \approx \left\| \mathrm{Log} \left(\pmb {T} _ {w} ^ {c ^ {0}} \pmb {Z} _ {i} ^ {- 1}\right) - \pmb {H} _ {i} \pmb {\xi} \right\| _ {\pmb {\Sigma} _ {i}} ^ {2},\tag{2.20}
$$

where $\mathbf { \mathcal { T } } _ { w } ^ { c }$ is perturbed as before and $\pmb { H } _ { i }$ is the appropriate Jacobian for this error [47]. In this way, measurements of any type can be handled in the factor-graph paradigm.

To recap, we have shown how to carry out unconstrained optimization for a state variable that is a member of a Lie group. Although our example was specific to a three-dimensional pose variable, other Lie groups can be optimized in a similar manner. The key is to arrive at a situation as in (2.16) where the measurement function has been linearized with respect to a perturbation in the Lie algebra. Most times, as in our example, this can be done analytically. However, it is also straightforward to compute the required Jacobian, $\pmb { H } _ { i }$ , numerically or through automatic diferentiation (by exploiting the chain rule and some primitives for Lie groups). Stepping back a bit, this approach to optimizing a function of a Lie group member is an example of Riemannian optimization [106]. By exploiting the Lie algebra, which is also the tangent space of a manifold, we constrain the optimization to be tangent to the manifold of poses (or rotations). By carrying out the update according to (2.19), we are retracting our update back onto the manifold. Riemannian optimization is a very general concept that can be applied to quantities that live on manifolds that are not matrix Lie groups as well. Retractions other than the matrix exponential are also possible within the manifold-optimization framework (e.g., see Dellaert et al. [255] or Barfoot et al. [49]).

## 2.1.4 Uncertainty and Lie Groups

We often represent uncertainty in our estimates by considering that the state vector is a random variable that follows some distribution. Gaussian distributions are the most common as discussed in Section 1.2.2. For a vector variable, x, we can write

$$
\boldsymbol {x} = \boldsymbol {\mu} + \boldsymbol {\delta}, \quad \boldsymbol {\delta} \sim \mathcal {N} (\boldsymbol {0}, \boldsymbol {\Sigma}),\tag{2.21}
$$

where $\pmb { \mu }$ is the mean and Σ is the covariance matrix. Notably, we have broken out the state into the sum of the mean and zero-mean noise, δ.

For Lie groups, we need to redefine how noise is combined with the state since simply adding noise to, for example, a rotation matrix $R \in \mathrm { S O } ( d )$ , will break the group closure property (i.e., the result will no longer be a valid rotation matrix). Instead, we typically use the surjective-only mapping of the matrix exponential to combine noise, δ, with a ‘mean’ quantity, $\bar { \pmb { R } } \in \mathrm { S O } ( d )$ , as follows:

$$
\boldsymbol {R} = \bar {\boldsymbol {R}} \oplus \boldsymbol {\delta} = \bar {\boldsymbol {R}} \mathrm{Exp} (\boldsymbol {\delta}), \quad \boldsymbol {\delta} \sim \mathcal {N} (\boldsymbol {0}, \boldsymbol {\Sigma}),\tag{2.22}
$$

where it is now guaranteed that $R \in \mathrm { S O } ( d )$ . A similar approach can be followed for $\operatorname { S E } ( d )$ :

$$
\boldsymbol {T} = \bar {\boldsymbol {T}} \oplus \boldsymbol {\delta} = \bar {\boldsymbol {T}} \operatorname{Exp} (\boldsymbol {\delta}), \quad \boldsymbol {\delta} \sim \mathcal {N} (\boldsymbol {0}, \boldsymbol {\Sigma}),\tag{2.23}
$$

where it is now guaranteed that $\pmb { T } \in \mathrm { S E } ( d )$ and naturally δ and Σ must be appropriately sized. To learn more see, for example, Long et al. [689], Barfoot and Furgale [48]. We remark that in (2.22) we essentially induced a noise distribution over rotations by defining a Gaussian distribution in the tangent space and mapping it to a rotation using the exponential map. However, it is also possible to directly define distributions over rotations, and in some cases this leads to computational advantages, as we will observe in Chapter 6.

## 2.1.5 Lie Group Extras

There is a lot more that we could say about Lie groups [1033, 106] but have so far restrained ourselves in the interest of keeping things simple. We use this section to collect a few more useful facts that come up later in this and other chapters. Further details of carrying out derivatives of functions of Lie group elements will be provided in Section 4.3.

## 2.1.5.1 The and Operators

We have already seen the use of the operator to compose a Lie algebra vector with a Lie group member. For $\operatorname { S E } ( d )$ we have

$$
\boldsymbol {T} = \boldsymbol {T} ^ {0} \oplus \boldsymbol {\xi} = \boldsymbol {T} ^ {0} \mathrm{Exp} (\boldsymbol {\xi}) = \boldsymbol {T} ^ {0} \exp (\boldsymbol {\xi} ^ {\wedge}) \in \mathrm{SE} (d).\tag{2.24}
$$

We often have occasion to consider the ‘diference’ of two Lie group elements and for this we can also define the $\ominus$ operator. Again for $\operatorname { S E } ( d )$ we have

$$
\boldsymbol {T} = \boldsymbol {T} ^ {0} \mathrm{Exp} (\boldsymbol {\xi}) \quad \rightarrow \quad \mathrm{Exp} (\boldsymbol {\xi}) = \boldsymbol {T} ^ {0 ^ {- 1}} \boldsymbol {T} \quad \stackrel {{\text { denoted   as: }}} {{\longrightarrow}} \quad \boldsymbol {\xi} = \boldsymbol {T} \ominus \boldsymbol {T} ^ {0},\tag{2.25a}
$$

$$
\boldsymbol {\xi} = \boldsymbol {T} \ominus \boldsymbol {T} ^ {0} = \operatorname{Log} \left(\boldsymbol {T} ^ {0 ^ {- 1}} \boldsymbol {T}\right) = \log \left(\boldsymbol {T} ^ {0 ^ {- 1}} \boldsymbol {T}\right) ^ {\vee} \in \operatorname{se} (d).\tag{2.25b}
$$

These operators are a nice way to abstract away the details of these operations.

## 2.1.5.2 Inverses

Sometimes when we are carrying out perturbations, we have need to perturb the inverse of a rotation or transformation matrix. In the $\operatorname { S E } ( d )$ case, we simply have that

$$
\left(\boldsymbol {T} ^ {0} \oplus \boldsymbol {\xi}\right) ^ {- 1} = \operatorname{Exp} (\boldsymbol {\xi}) ^ {- 1} \boldsymbol {T} ^ {0 ^ {- 1}} = \operatorname{Exp} (- \boldsymbol {\xi}) \boldsymbol {T} ^ {0 ^ {- 1}} = (- \boldsymbol {\xi}) \oplus \boldsymbol {T} ^ {0 ^ {- 1}},\tag{2.26}
$$

where we see the perturbation moves from the right to the left with a negative sign.

## 2.1.5.3 Adjoints

The adjoint of a Lie group is a way of describing the elements of that group as linear transformations of its Lie algebra, which we recall is a vector space. For $\mathrm { S O } ( d )$ , the adjoint representation is the same as the group itself, so we omit the details. For $\operatorname { S E } ( d )$ , the adjoint difers from the group’s primary representation and so we use this section to provide some details. The adjoint will prove to be an essential tool when setting up state estimation problems, particularly for $\operatorname { S E } ( d )$

The adjoint map of $\operatorname { S E } ( d )$ for a given pose T transforms a Lie algebra element ${ \pmb { \xi } } ^ { \wedge } \in \mathrm { s e } ( d )$ , in the local frame of $\mathbf { T }$ , to another element $\epsilon ^ { \wedge } \in \mathrm { s e } ( d )$ in the global frame, that is,

$$
\boldsymbol {\epsilon} ^ {\wedge} \oplus \boldsymbol {T} = \boldsymbol {T} \oplus \boldsymbol {\xi} ^ {\wedge}\tag{2.27}
$$

according to a map known as the inner automorphism or conjugation:

$$
\boldsymbol {\epsilon} ^ {\wedge} = \operatorname{Ad} _ {T} \boldsymbol {\xi} ^ {\wedge} = T \boldsymbol {\xi} ^ {\wedge} T ^ {- 1}.\tag{2.28}
$$

We can equivalently express the output of this map using the Adjoint matrix $\operatorname { A d } ( \pmb { T } )$ that linearly transforms $\pmb { \xi } \in \mathbb { R } ^ { 6 }$ to $\mathbb { R } ^ { 6 }$ , such as:

$$
\operatorname{Ad} _ {\boldsymbol {T}} \boldsymbol {\xi} ^ {\wedge} = (\operatorname{Ad} (\boldsymbol {T}) \boldsymbol {\xi}) ^ {\wedge}.\tag{2.29}
$$

We will refer to $\operatorname { A d } ( \pmb { T } )$ as the adjoint representation of $\operatorname { S E } ( d )$ at $\mathbf { T } .$

The $( 2 d ) \times ( 2 d )$ transformation matrix, $\operatorname { A d } ( \pmb { T } )$ , can be constructed directly from the components of the $( d + 1 ) \times ( d + 1 )$ homogeneous transformation matrix:

$$
\operatorname{Ad} (\boldsymbol {T}) = \operatorname{Ad} \left(\left[ \begin{array}{c c} \boldsymbol {R} & \boldsymbol {t} \\ \boldsymbol {0} & 1 \end{array} \right]\right) = \left[ \begin{array}{c c} \boldsymbol {R} & \boldsymbol {t} ^ {\wedge} \boldsymbol {R} \\ \boldsymbol {0} & \boldsymbol {R} \end{array} \right].\tag{2.30}
$$

One situation in which adjoints are useful in our estimation problems is to manip ulate perturbations from one side of a known transformation to another as in

$$
\boldsymbol {T} \operatorname{Exp} (\boldsymbol {\xi}) = \operatorname{Exp} \left(\operatorname{Ad} (\boldsymbol {T}) \boldsymbol {\xi}\right) \boldsymbol {T},\tag{2.31}
$$

which we emphasize does not require approximation.

## 2.1.5.4 Jacobians

Every Lie group also has a Jacobian associated with it, which allows us to relate changes in an element of the group to elements of its algebra. For the case of $\mathrm { S O } ( d )$ , for example, the common kinematic equation $( i . e .$ , Poisson’s equation) relating a rotation matrix, $R _ { a } ^ { b } \in \mathrm { S O } ( d )$ for d either 2 or 3, to angular velocity, $\omega ^ { b } \in \mathbb { R } ^ { 2 d - 3 }$ , is

$$
\dot {\boldsymbol {R}} _ {a} ^ {b} = \boldsymbol {\omega} ^ {b ^ {\wedge}} \boldsymbol {R} _ {a} ^ {b}.\tag{2.32}
$$

Note also that $\omega ^ { b }$ represents the angular velocity of ${ \mathcal { F } } ^ { a }$ with respect to $\mathcal { F } ^ { b }$ , expressed in $\mathcal { F } ^ { b }$ . If we parameterize $R _ { a } ^ { b } = \exp ( \theta )$ , then we can equivalently write

$$
\dot {\boldsymbol {\theta}} = \boldsymbol {J} ^ {- 1} (\boldsymbol {\theta}) \boldsymbol {\omega} ^ {b},\tag{2.33}
$$

where $\pmb { J } ( \pmb \theta )$ is the (left) Jacobian of $\mathrm { S O } ( d )$ . A place where this Jacobian is quite useful is when combining expressions involving products of matrix exponentials. For example, we have that

$$
\mathrm{Exp} (\pmb {\theta} _ {1}) \mathrm{Exp} (\pmb {\theta} _ {2}) \approx \mathrm{Exp} \left(\pmb {\theta} _ {2} + \pmb {J} (\pmb {\theta} _ {2}) ^ {- 1} \pmb {\theta} _ {1}\right),\tag{2.34}
$$

where $\pmb { \theta } _ { 1 }$ is assumed to be ‘small’. The series expression for $\pmb { J } ( \pmb \theta )$ is

$$
\boldsymbol {J} (\boldsymbol {\theta}) = \sum_ {n = 0} ^ {\infty} \frac {1}{(n + 1) !} \left(\boldsymbol {\theta} ^ {\wedge}\right) ^ {n},\tag{2.35}
$$

and a closed-form expression can be found in Barfoot [47]. We will overload and write $\pmb { J } ( \pmb { \xi } )$ for the (left) Jacobian of $\operatorname { S E } ( d )$ where the context should inform which is meant. Jacobians of Lie groups are at the core of diferentiable neural network methods that estimate poses (see Section 4.2).

## 2.2 Continuous-Time Trajectories

Continuous-time trajectories ofer a way to represent smooth robot motions. In our development so far, we have assumed that a discrete sequence of poses along a trajectory is to be estimated. However, robots typically move fairly smoothly through the world, which motivates the use of a smoother representation of trajectory. Continuous-time trajectories come primarily in two varieties: parametric methods combine known temporal basis functions into a smooth trajectory. Typically, these temporal basis functions are chosen to have local support $( e . g .$ , piecewise polynomials / splines), which ensures the factor graph remains sparse, as we will see. Nonparametric methods have higher representational power by making use of kernel functions. Specifically, a one-dimensional Gaussian process $( G P )$ with time as the independent variable can be used to represent a trajectory. When an appropriate physically motivated kernel is chosen, we will see that the factor graph associated with a GP also remains very sparse.

In addition to trajectory smoothness, the use of a continuous-time trajectory can be particularly useful when working with high-rate (such as inertial measurement units; see Section 11.2) and/or asynchronous sensors. In the factor-graph examples that we have considered so far, we added robot poses to the factor graph for each newly collected measurement (e.g., to model that the current pose is taking a landmark measurement). This quickly leads to unwieldy factor graphs when using high-rate sensors or when diferent sensors collect measurements at diferent time instants. Below, we will see that we can easily represent the trajectory with a number of variables that is much smaller than the number of measurements, to keep things tractable. This is particularly useful for motion-distorted sensors such as spinning lidars (see Section 8.2.2.1) and radars and even rolling-shutter cameras; using continuous-time trajectories we can account for the exact time stamp of each point or pixel and relate them to the trajectory at that instant.

Finally, after MAP inference, continuous-time trajectories allow us to eficiently query the trajectory at any time of interest, not just at the measurement times. We can both interpolate and extrapolate (with caution), which can be useful for consumers of our SLAM outputs. Separating the roles of measurements times, estimation variables, and query times, is a major advantage of both parametric and nonparametric continuous-time methods.

## 2.2.1 Splines

The idea with parametric continuous-time trajectory methods is to write the pose as a weighted sum of K known temporal basis functions, $\Psi _ { k } ( t )$ :

$$
\boldsymbol {p} (t) = \sum_ {k = 1} ^ {K} \boldsymbol {\Psi} _ {k} (t) \boldsymbol {c} _ {k},\tag{2.36}
$$

where the $\scriptstyle c _ { k }$ are the unknown coeficients. For now, we return to a vector-space explanation and discuss implementation on Lie groups in a later section. The basis functions are typically chosen to be splines, which are piecewise polynomials (e.g., B-splines, cubic Hermite polynomials); splines are advantageous because they have local support meaning outside of their local region of influence they go to zero. The setup is depicted in Figure 2.3. In this example, at each instant of time only four basis functions are nonzero, which we see results in a sparse factor graph.

The main diference, as compared to our earlier discrete-time development, is that we have coeficient variables instead of pose variables, but this is completely compatible with the general factor-graph approach. Now, when we observe a landmark, ℓ, at a particular time, $t _ { i }$ , the sensor model is

$$
\boldsymbol {z} _ {i} = \boldsymbol {h} _ {i} (\boldsymbol {p} (t _ {i}), \boldsymbol {\ell}) + \boldsymbol {\eta} _ {i}.\tag{2.37}
$$

temporal basis functions  
![](images/abdd8e7b3af2c35776787d1be6ccefd148773139241eea52f9e85dbe4af53cf9.jpg)  
Figure 2.3 A parametric spline can be used to represent a continuous-time trajectory. In this example, the pose at a given time ${ \pmb p } ( t )$ is assembled as a weighted sum of known temporal basis functions $\Psi _ { k } ( t )$ with local support; at most four basis functions are nonzero at a given time. This results in each landmark measurement being represented by a quinary (five-way) factor between four coeficient variables and one landmark variable. The overall factor graph is still very sparse.

Inserting (2.36) we have

$$
\boldsymbol {z} _ {i} = \boldsymbol {h} _ {i} \left(\sum_ {k = 1} ^ {K} \Psi_ {k} (t _ {i}) \boldsymbol {c} _ {k}, \ell\right) + \boldsymbol {\eta} _ {i}.\tag{2.38}
$$

As mentioned above, if our basis functions are chosen to have local support, then only a small subset of the coeficients will be active at $t _ { i } .$ If we let $\begin{array} { r } { \pmb { x } _ { i } = \left[ \pmb { c } _ { i } ^ { \top } \quad \pmb { \ell } ^ { \top } \right] ^ { \top } } \end{array}$ represent the active coeficient variables at $t _ { i }$ as well as the landmark variable, then we are back to being able to write the measurement function as

$$
\boldsymbol {z} _ {i} = \boldsymbol {h} _ {i} (\boldsymbol {x} _ {i}) + \boldsymbol {\eta} _ {i},\tag{2.39}
$$

whereupon we can use our general approach to construct the nonlinear least-squares problem and optimize.

Moreover, if our basis functions are suficiently diferentiable, we can easily take the derivative of our pose trajectory,

$$
\dot {\pmb {p}} (t) = \sum_ {k = 1} ^ {K} \dot {\pmb {\Psi}} _ {k} (t) \pmb {c} _ {k}\tag{2.40}
$$

so that we can handle sensor outputs that are functions of, say, velocity or even higher derivatives while still optimizing the same coeficient variables. We simply need to compute the derivatives of our basis functions, $\dot { \Psi } _ { k } ( t )$

Finally, once we have solved for the optimal coeficients through MAP inference, we can then query the trajectory (or its derivatives) at any time of interest using (2.36) or (2.40). If we compute the covariance of the estimated coeficients during inference (e.g., by inverting the information matrix), this can also be mapped through to covariance of a queried pose (or derivative) quite easily since (2.36) or (2.40) are linear relationships; and, local support in the basis functions implies only the appropriate marginal covariance is needed from the coeficients.

## 2.2.2 From Parametric to Nonparametric

The main challenge with basic parametric continuous-time methods is that we must decide what type and how many basis functions to use. If we have too many basis functions, it becomes very easy to overfit to the measurement data. If we have too few basis functions, we may not have suficient capacity to represent the true shape of the trajectory, resulting in an overly smooth solution. This challenge is partly addressed by moving to a nonparametric method.

To simplify the explanation slightly, in this section we will assume for now that there are no landmark variables, and focus only on pose variables. Using the parametric approach introduced in the previous section, our linearized least-squares term (negative-log factor) will have the form

$$
\left\| \left(\boldsymbol {z} _ {i} - \boldsymbol {h} _ {i} \left(\boldsymbol {x} _ {i} ^ {0}\right)\right) - \boldsymbol {H} _ {i} \boldsymbol {\Psi} _ {i} \delta_ {c, i} \right\| _ {\boldsymbol {\Sigma} _ {i}} ^ {2},\tag{2.41}
$$

where $\pmb { x } _ { i } ^ { 0 }$ is the current solution (active coeficients), $\delta _ { c , i }$ is the update (to the active coeficients), $\Psi _ { i }$ is the stacking of all basis functions active (and evaluted) at $t _ { i } ,$ and the Jacobian, $\pmb { H } _ { i }$ , is given by

$$
\boldsymbol {H} _ {i} = \left. \frac {\partial \boldsymbol {h} _ {i}}{\partial \boldsymbol {x}} \right| _ {\boldsymbol {x} _ {i} ^ {0}}.\tag{2.42}
$$

Gathering quantities into larger matrices as before, we can write our least-squares problem as

$$
\boldsymbol {\delta} _ {c} ^ {*} = \underset {\boldsymbol {\delta} _ {c}} {\arg \min} \left(\| \boldsymbol {b} - \boldsymbol {A} \boldsymbol {\Psi} \boldsymbol {\delta} _ {c} \| ^ {2} + \| \boldsymbol {\delta} _ {c} \| ^ {2}\right),\tag{2.43}
$$

where we now include a regularizer term, $\left\| \delta _ { c } \right\| ^ { 2 }$ , that seeks to keep the description length of our solution reasonable $( i . e .$ , we prefer spline coeficients to be closer to zero). The regularizer term helps to avoid the over-fitting problem mentioned above. The optimal solution will be given by

$$
\left(\boldsymbol {\Psi} ^ {\top} \boldsymbol {A} ^ {\top} \boldsymbol {A} \boldsymbol {\Psi} + \mathbf {I}\right) \delta_ {c} ^ {*} = \boldsymbol {\Psi} ^ {\top} \boldsymbol {A} ^ {\top} \boldsymbol {b},\tag{2.44}
$$

which would allow us to compute the optimal update for the coeficients, $\delta _ { c } ^ { * }$ . However, what we typically care about is to produce an estimate for the pose, not the spline coeficients (they are a means to an end). The optimal update to the pose variables at the measurement times is actually $\delta ^ { * } = \Psi \delta _ { c } ^ { * }$ . With a little bit of algebra, we can show that

$$
\left(\boldsymbol {A} ^ {\mathsf {T}} \boldsymbol {A} + \boldsymbol {K} ^ {- 1}\right) \boldsymbol {\delta} ^ {*} = \boldsymbol {A} ^ {\mathsf {T}} \boldsymbol {b},\tag{2.45}
$$

which is a modified version of the normal equations, first introduced in (1.25). The kernel matrix, $K = \Psi \Psi ^ { \mathsf { T } }$ , serves as a regularization or smoothing function. The careful reader will notice that (2.45) represents a larger linear system of equations than (2.44) because there are more poses than basis function coeficients. However, in the end we will be able to reduce the size of the linear system we need to solve in our nonparametric approach by using built-in interpolation capabilities. For now, we will work with (2.45) and come back to this issue towards the end of the section.

To move away from explicit basis functions, we can employ the so-called kernel trick, which replaces the explicit inner product of basis functions with evaluations of a chosen kernel function, $\mathcal { K } ( t , t ^ { \prime } ) \ \left( e . g . \right.$ , squared-exponential). We can see that in (2.45) it is only the inner product of the basis functions that is required to build the kernel matrix. The kernel matrix is then $\mathbf { } K = \left[ \mathcal { K } ( t _ { i } , t _ { j } ) \right] _ { i j } .$ , which is to say we populate it with evaluations of the kernel function at every pairing of measurement times. We can now refer to this as a nonparametric method since we are no longer estimating the coeficients (i.e., parameters) of a spline. We $\mathrm { d o } ,$ , however, have to tune the hyperparameters of our chosen kernel function (e.g., length scale for squared exponential) to achieve the desired trajectory smoothness.

Since we need the inverse kernel matrix right away in (2.45), it would seem to be expensive to formulate things this way. However, the next section shows how we can choose a kernel function that guarantees that we have a very sparse inverse kernel matrix and therefore a sparse factor graph.

## 2.2.3 Gaussian Processes

We will construct a family of kernel functions that by design results in a sparse inverse kernel matrix and corresponding factor graph. We saw in the last section that we could swap out our basis functions for a kernel function, creating a nonparametric continuous-time method. However, if done naively, this could result in a dense inverse kernel matrix, which is undesirable. In this section, we come at things from a slightly diferent direction. As a teaser, Figure 2.4 shows an example of a factor graph resulting from the ideas in this section, which we see remains sparse yet results in smooth trajectories.

We start by choosing a linear, time-invariant, stochastic diferential equation (SDE) driven by white noise:<sup>2</sup>

$$
\dot {\boldsymbol {x}} (t) = \boldsymbol {A} \boldsymbol {x} (t) + \boldsymbol {L} \boldsymbol {w} (t),\tag{2.46}
$$

![](images/c085cb8fe22faf2a155c7a521372d21ce6b7b2792d91fd9afca43cdcc65857fe.jpg)  
Figure 2.4 Example of Gaussian process (GP) continuous-time factor graph. The motion prior is based on a kernel function derived from a stochastic diferential equation (SDE) for Markovian state ${ \pmb x } ( t )$ . This results in a very sparse set of factors: a single unary factor at the initial state and then binary factors linking consecutive states.

where ${ \pmb w } ( t ) = \mathcal { G P } ( { \bf 0 } , \pmb { \mathcal { Q } } \delta ( t - t ^ { \prime } ) )$ is a zero-mean white noise Gaussian process, $\mathfrak { Q }$ is a power-spectral density matrix, and $\delta ( \cdot )$ is the Dirac delta function. The idea is that this will serve as a motion prior. We can integrate this SDE once in closed form:

$$
\boldsymbol {x} (t) = \boldsymbol {\Phi} (t, t _ {1}) \boldsymbol {x} (t _ {1}) + \int_ {t _ {1}} ^ {t} \boldsymbol {\Phi} (t, s) \boldsymbol {L} \boldsymbol {w} (s) d s,\tag{2.47}
$$

where $\Phi ( t , s ) = \exp \left( A ( t - s ) \right)$ is known as the transition function and $t _ { 1 }$ is the time stamp of the first measurement. The function, ${ \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf } { \mathbf { } } { \mathbf { } } { \mathbf } { } \mathbf { }  { \mathbf { } \mathbf { } } { \mathbf { } \mathbf { } } { \mathbf } { \mathbf { } } { \mathbf } { \mathbf { } } { \mathbf } { \mathbf } { } \mathbf { } \mathbf { } \mathbf { } \mathbf { }  { \mathbf } { \mathbf } { \mathbf } { \mathbf } { \mathbf } { \mathbf } { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf  { \mathbf \mathbf { } \mathbf { } \mathbf } { \mathbf \mathbf { } \mathbf { } \mathbf } { \mathbf } { \mathbf \mathbf { } \mathbf } \mathbf { \mathbf } { \mathbf } \mathbf  $ , is also a Gaussian process. To keep the explanation simple, if we assume the mean of the initial state is zero, $E [ { \pmb x } ( t _ { 1 } ) ] = { \bf 0 }$ , then the mean will remain zero for all subsequent times. The covariance function of the state $( i . e .$ , the kernel function), $\kappa ( t , t ^ { \prime } )$ , can be calculated as

$$
\boldsymbol {\mathcal {K}} (t, t ^ {\prime}) = \boldsymbol {\Phi} (t, t _ {1}) \boldsymbol {\mathcal {K}} (t _ {1}, t _ {1}) \boldsymbol {\Phi} (t ^ {\prime}, t _ {1}) ^ {\top} + \int_ {t _ {1}} ^ {\min (t, t ^ {\prime})} \boldsymbol {\Phi} (t, s) \boldsymbol {L} \boldsymbol {Q} \boldsymbol {L} ^ {\top} \boldsymbol {\Phi} (t ^ {\prime}, s) ^ {\top} d s,\tag{2.48}
$$

which looks daunting. However, we can evaluate this kernel function at all pairs of measurement times $( i . e .$ , build the kernel matrix) using the tidy relation

$$
\boldsymbol {K} = \boldsymbol {\Phi} \boldsymbol {Q} \boldsymbol {\Phi} ^ {\mathsf {T}},\tag{2.49}
$$

where $\begin{array} { r } { Q = \mathrm { d i a g } ( \mathcal { K } ( t _ { 1 } , t _ { 1 } ) , Q _ { 1 } , . . . , Q _ { M } ) , Q _ { i } = \int _ { t _ { i - 1 } } ^ { t _ { i } } \Phi ( t _ { i } , s ) L Q L ^ { \top } \Phi ( t _ { i } , s ) ^ { \top } } \end{array}$ ds, and

$$
\boldsymbol {\Phi} = \left[ \begin{array}{c c c c c c} \mathbf {I} & & & & \\ \boldsymbol {\Phi} (t _ {2}, t _ {1}) & \mathbf {I} & & & \\ \boldsymbol {\Phi} (t _ {3}, t _ {1}) & \boldsymbol {\Phi} (t _ {3}, t _ {2}) & \mathbf {I} & & \\ \vdots & \vdots & \vdots & \ddots & \\ \boldsymbol {\Phi} (t _ {M - 1}, t _ {1}) & \boldsymbol {\Phi} (t _ {M - 1}, t _ {2}) & \boldsymbol {\Phi} (t _ {M - 1}, t _ {3}) & \dots & \mathbf {I} \\ \boldsymbol {\Phi} (t _ {M}, t _ {1}) & \boldsymbol {\Phi} (t _ {M}, t _ {2}) & \boldsymbol {\Phi} (t _ {M}, t _ {3}) & \dots & \boldsymbol {\Phi} (t _ {M}, t _ {M - 1}) & \mathbf {I} \end{array} \right],\tag{2.50}
$$

with M the last measurement time index. However, since it is the inverse kernel matrix that we want in (2.45), $K ^ { - 1 } = \Phi ^ { - \top } Q ^ { - 1 } \Phi ^ { - 1 }$ , we can compute this directly. The middle matrix, $Q ,$ is block-diagonal and so its inverse can be computed one diagonal block at a time. Importantly, when we compute the inverse of Φ, we find

$$
\boldsymbol {\Phi} ^ {- 1} = \left[ \begin{array}{c c c c c c} \mathbf {I} & & & & & \\ - \boldsymbol {\Phi} (t _ {2}, t _ {1}) & \mathbf {I} & & & & \\ & - \boldsymbol {\Phi} (t _ {3}, t _ {2}) & \mathbf {I} & & & \\ & & - \boldsymbol {\Phi} (t _ {4}, t _ {3}) & \ddots & & \\ & & & \ddots & \mathbf {I} \\ & & & & - \boldsymbol {\Phi} (t _ {M}, t _ {M - 1}) & \mathbf {I} \end{array} \right],\tag{2.51}
$$

which is all zeros except for the main block-diagonal and one block-diagonal below. Thus, when we construct the inverse kernel matrix, $\pmb { K } ^ { - 1 }$ , it will be block-tridiagonal, for any length of trajectory. Based on our earlier discussions about factor graphs, we know that the sparsity of the left-hand side in (2.45) is closely tied to the factor-graph structure. In this case, $\pmb { K } ^ { - 1 }$ serves as a motion prior over the entire trajectory, but it is easily described using a very sparse factor graph. Figure 2.4 shows how the block-tridiagonal structure of $\pmb { K } ^ { - 1 }$ turns into a factor graph.

The reason $\pmb { K } ^ { - 1 }$ has such a sparse factor graph is that we started from an SDE whose state, ${ \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf } { \mathbf { } } { \mathbf { } } { \mathbf } { } \mathbf { }  { \mathbf { } \mathbf { } } { \mathbf { } \mathbf { } } { \mathbf } { \mathbf { } } { \mathbf } { \mathbf { } } { \mathbf } { \mathbf } { } \mathbf { } \mathbf { } \mathbf { } \mathbf { }  { \mathbf } { \mathbf } { \mathbf } { \mathbf } { \mathbf } { \mathbf } { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf  { \mathbf \mathbf { } \mathbf { } \mathbf { } \mathbf } { \mathbf } { \mathbf } \mathbf { \mathbf } { \mathbf } \mathbf { } \mathbf { \mathbf } \mathbf { } \mathbf \mathbf  $ , is Markovian. Practically speaking, what this means is that depending on the motion prior that we want to express using (2.46), we may need to use a higher-order state, i.e., not simply the pose but also some of its derivatives. For example, if we want to use the so-called ‘constant-velocity’ prior, our SDE can be chosen to be

$$
\underbrace {\left[ \begin{array}{c} \dot {\boldsymbol {p}} (t) \\ \dot {\boldsymbol {v}} (t) \end{array} \right]} _ {\dot {\boldsymbol {x}} (t)} = \underbrace {\left[ \begin{array}{c c} \mathbf {0} & \mathbf {I} \\ \mathbf {0} & \mathbf {0} \end{array} \right]} _ {\boldsymbol {A}} \underbrace {\left[ \begin{array}{c} \boldsymbol {p} (t) \\ \boldsymbol {v} (t) \end{array} \right]} _ {\boldsymbol {x} (t)} + \underbrace {\left[ \begin{array}{c} \mathbf {0} \\ \mathbf {I} \end{array} \right]} _ {\boldsymbol {L}} \boldsymbol {w} (t),\tag{2.52}
$$

where the state now comprises pose and its derivative, $\mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { }  \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf \mathbf { } \mathbf { } \mathbf { } \mathbf \mathbf { } \mathbf { } \mathbf \mathbf { } \mathbf { } \mathbf \mathbf { } \mathbf \mathbf { } \mathbf \mathbf { } \mathbf \mathbf { } \mathbf \mathbf { } \mathbf \mathbf { } \mathbf \mathbf \mathbf { } \mathbf \mathbf { } \mathbf \mathbf \mathbf { } \mathbf \mathbf \mathbf { } \mathbf \mathbf $ . Due to the use of this augmented state, this is sometimes this is referred to as simultaneous trajectory estimation and mapping (STEAM), a variation of SLAM.

This formulation of continuous-time trajectory estimation is really an example of Gaussian process regression [909]. By making this connection, once we have solved at the measurement times, we can easily query the trajectory at other times of interest using GP interpolation (for both mean and covariance); with our sparse kernel approach, the cost of each query is constant time with respect to the number of measurements, M, as it only involves the estimated states at the two times bracketing the query.

Importantly, we can also use the resulting GP interpolation scheme to reduce the number of control points needed $( i . e .$ , we do not need one at every measurement time), which is similar to the idea of $G P$ inducing points. For example, we might put one control point per lidar scan but still make use of all the individual time stamps of each point gathered during a sweep. This last point is quite important because in contrast to discrete-time estimation, the measurement times, the estimation times, and the query times can now all be diferent in this continuous formulation. Moreover, in the GP approach we do not need to worry about overfitting by including too many estimation times as the kernel provides proper regularization. However, we still need enough estimation times to capture the detail of the trajectory.

## 2.2.4 Spline and GPs on Lie Groups

It is also possible to use both splines and GP continuous-time methods when the state lives on a manifold. In the case that the manifolds are Lie groups, both methods make use of the Lie algebra to accomplish this, but in diferent ways. We begin with splines and then move to Gaussian processes.

## 2.2.4.1 Splines on Lie Groups

The key to making splines work on Lie groups is to use a cumulative formulation. For a vector space, we can simplify (2.36) by assuming we are using the same basis functions for all degrees of freedom so that we can write

$$
\boldsymbol {p} (t) = \sum_ {k = 1} ^ {K} \psi_ {k} (t) \boldsymbol {p} _ {k},\tag{2.53}
$$

where the ${ \pmb p } _ { k }$ are now control points of our spline (replacing the earlier coeficients) and the basis functions, $\psi _ { k } ( t )$ , are now scalar. Then, we can rewrite this in cumulative form as

$$
\boldsymbol {p} (t) = \psi_ {1} ^ {c} (t) \boldsymbol {p} _ {1} + \sum_ {k = 2} ^ {K} \psi_ {k} ^ {c} (t) \left(\boldsymbol {p} _ {k} - \boldsymbol {p} _ {k - 1}\right),\tag{2.54}
$$

where

$$
\psi_ {k} ^ {c} (t) = \sum_ {\ell = k} ^ {K} \psi_ {\ell} (t)\tag{2.55}
$$

are the cumulative basis functions.

linear spline basis functions  
![](images/f22dd9a05233a84c0d90646b480aa632adafc84c5b3a414c42055a9c34c19576.jpg)  
linear spline cumulative basis functions  
Figure 2.5 Example of linear spline basis functions both in (top) normal and (bottom) cumulative form.

For example, if we want to have linear interpolation with uniform temporal spacing, T, the basis functions are

$$
\psi_ {1} (t) = \left\{ \begin{array}{l l} 1 - \alpha_ {1} (t) & 0 \leq t <   T \\ 0 & \text {otherwise} \end{array} \right., \quad \psi_ {K} (t) = \left\{ \begin{array}{l l} \alpha_ {K} (t) & (K - 1) T \leq t <   K T \\ 0 & \text {otherwise} \end{array} \right.\tag{2.56}
$$

$$
k = 2 \ldots K - 1: \quad \psi_ {k} (t) = \left\{ \begin{array}{l l} \alpha_ {k - 1} (t) & (k - 2) T \leq t <   (k - 1) T \\ 1 - \alpha_ {k} (t) & (k - 1) T \leq t <   k T \\ 0 & \text { otherwise } \end{array} \right.,\tag{2.57}
$$

where $\begin{array} { r } { \alpha _ { k } ( t ) = \frac { t - ( k - 1 ) T } { T } } \end{array}$ . The corresponding cumulative basis functions are

$$
\psi_ {1} ^ {c} (t) = 1, \quad \psi_ {K} ^ {c} (t) = \left\{ \begin{array}{l l} 0 & \leq t <   (K - 1) T \\ \alpha_ {k} (t) & (k - 1) T \leq t \end{array} \right.,\tag{2.58}
$$

$$
k = 2 \ldots K - 1: \quad \psi_ {k} ^ {c} (t) = \left\{ \begin{array}{l l} 0 & t <   (k - 1) T \\ \alpha_ {k} (t) & (k - 1) T \leq t <   k T \\ 1 & k T \leq t \end{array} \right..\tag{2.59}
$$

Figure 2.5 shows what these basis functions look like.

The key advantage of the cumulative basis functions is that at a given time stamp, most of the basis functions are inactive. In the case of our linear spline example, we can write

$$
\boldsymbol {p} (t) = \boldsymbol {p} _ {k - 1} + \psi_ {k} ^ {c} (t) (\boldsymbol {p} _ {k} - \boldsymbol {p} _ {k - 1})\tag{2.60}
$$

when $( k - 1 ) T \leq t < k T$ . We see that only a single basis function needs to be evaluated. With higher-order splines, we will still have only a small active set at a particular time stamp.

To apply splines on a Lie group, the idea is to then use the cumulative formulation with the Lie group operator (matrix multiplication) replacing the summation. For example, in the case of a linear spline, an element of $\operatorname { S E } ( d )$ can be written as

$$
\boldsymbol {T} (t) = \mathrm{Exp} \left(\psi_ {k} ^ {c} (t) \mathrm{Log} \left(\boldsymbol {T} _ {k} \boldsymbol {T} _ {k - 1} ^ {- 1}\right)\right) \cdot \boldsymbol {T} _ {k - 1},\tag{2.61}
$$

when $( k - 1 ) T \leq t < k T$ . Note, we have chosen to carry out perturbations on the left-hand side this time, but a similar formulation with right perturbations is also possible. We can now insert $\pmb { T } ( t _ { i } )$ into any measurement expression at some time stamp $t _ { i } ,$ linearize it with respect to the $\mathbf { \delta } _ { \mathbf { \mathcal { T } } _ { k } }$ control points (our estimation variables), and then use it within our MAP framework. Again, with compact-support basis functions, only a few are active at a given measurement time (one in the example of linear splines).

In a bit more detail for our linear spline example, we can rewrite (2.61) as

$$
\boldsymbol {T} (t) = \left(\boldsymbol {T} _ {k} \boldsymbol {T} _ {k - 1} ^ {- 1}\right) ^ {\alpha_ {k} (t)} \boldsymbol {T} _ {k - 1}.\tag{2.62}
$$

When linearizing expressions involving $\pmb { T } ( t )$ , we can make use of the optimization approach introduced in Section 2.1.3. We perturb each of the $\mathrm { p o s e s ^ { 3 } }$ so that

$$
\operatorname{Exp} (\boldsymbol {\xi} (t)) \boldsymbol {T} ^ {0} (t) = \left(\operatorname{Exp} (\boldsymbol {\xi} _ {k}) \boldsymbol {T} _ {k} ^ {0} \boldsymbol {T} _ {k - 1} ^ {0 ^ {- 1}} \operatorname{Exp} (- \boldsymbol {\xi} _ {k - 1})\right) ^ {\alpha_ {k} (t)} \operatorname{Exp} (\boldsymbol {\xi} _ {k - 1}) \boldsymbol {T} _ {k - 1} ^ {0}.\tag{2.63}
$$

Our goal is to relate the perturbation of the interpolated pose, $\pmb \xi ( t )$ , to those of the control points, $\xi _ { k }$ and $\xi _ { k - 1 }$ . As shown by Barfoot [47], this relationship can be approximated (to first order in the perturbations) as

$$
\boldsymbol {\xi} (t) \approx (\boldsymbol {I} - \boldsymbol {A} (\alpha_ {k} (t))) \boldsymbol {\xi} _ {k - 1} + \boldsymbol {A} (\alpha_ {k} (t)) \boldsymbol {\xi} _ {k},\tag{2.64}
$$

where

$$
\boldsymbol {A} \left(\alpha_ {k} (t)\right) = \alpha_ {k} (t) \boldsymbol {J} \left(\alpha_ {k} (t) \boldsymbol {T} _ {k} ^ {0} \boldsymbol {T} _ {k - 1} ^ {0 ^ {- 1}}\right) \boldsymbol {J} \left(\boldsymbol {T} _ {k} ^ {0} \boldsymbol {T} _ {k - 1} ^ {0 ^ {- 1}}\right) ^ {- 1}\tag{2.65}
$$

and $J ( \cdot )$ is the left Jacobian of $\operatorname { S E } ( d )$ . We can then use (2.64) to relate changes in our pose at a measurement time to the two bracketing control-point poses in order to form linearized error terms for use in MAP estimation. For example, consider the linearized measurement model in (2.16) again, where we rearrange it as an error with slightly simpler notation for the pose and its perturbation as a function of t:

$$
\boldsymbol {e} _ {i} (t) \approx \boldsymbol {z} _ {i} - \boldsymbol {h} _ {i} \left(\boldsymbol {T} ^ {0} (t) \tilde {\ell} _ {i}\right) - \boldsymbol {H} _ {i} \boldsymbol {\xi} (t).\tag{2.66}
$$

Advanced State Variable Representations

![](images/79c7a98b5acf59b8f3ad3b3d2d1d964849a8d127b8bf7f35d2fbfd1746456268.jpg)  
Figure 2.6 Example GP motion prior factors when using a ‘random walk’ model.

It is now a simple matter of substituting (2.64) in for $\pmb \xi ( t )$ to produce a linearized error in terms of the bracketing control points:

$$
\pmb {e} _ {i} (t) \approx \pmb {z} _ {i} - \pmb {h} _ {i} \left(\pmb {T} (t) ^ {0} \tilde {\pmb {\ell}} _ {i}\right) - \pmb {H} _ {i} \left(\pmb {I} - \pmb {A} (\alpha_ {k} (t))\right) \pmb {\xi} _ {k - 1} - \pmb {H} _ {i} \pmb {A} (\alpha_ {k} (t)) \pmb {\xi} _ {k}.\tag{2.67}
$$

Note, we also need to substitute $\pmb { T } ^ { 0 } ( t ) = \left( \pmb { T } _ { k } ^ { 0 } \pmb { T } _ { k - 1 } ^ { 0 ^ { - 1 } } \right) ^ { \alpha _ { k } ( t ) } \pmb { T } _ { k - 1 } ^ { 0 }$ for the nominal pose at t, both within $\boldsymbol { h } _ { i }$ and $H _ { i } .$ . We have essentially chained the derivative through our linear spline. The same process can be followed for higher-order splines as well.

## 2.2.4.2 Gaussian Processes on Lie Groups

To use Gaussian processes on a Lie group, we will again exploit its Lie algebra to do so. Figure 2.6 provides a visual teaser of the GP motion-prior factors resulting from the ideas in this section. Note, as in the vector-space case, depending on the chosen motion prior, the control-point state may comprise additional trajectory derivatives as well.

To apply GPs on a Lie group, we will employ a local GP between a set of controlpoint states [47], similar to splines. Figure 2.7 provides a depiction of these local variables for $\operatorname { S E } ( d )$ . This means the SDE used to derive our kernel function operates on these local variables. For example, in the case of a ‘random-walk’ prior for $\operatorname { S E } ( d )$ , we could choose the SDE to be

![](images/5c0e03287543c7ed77db6aef2f3e2a0efd68dbeae3ca3b23fd7ef776f331b9ed.jpg)  
Figure 2.7 When using a GP for continuous-time estimation on Lie groups $( e . g . , \mathrm { S E } ( d ) )$ ), a local variable, ξ<sub>k</sub>(t), is defined between control-point states.

$$
\dot {\pmb {\xi}} _ {k} (t) = \pmb {w} (t), \quad \pmb {w} (t) = \mathcal {G P} (\pmb {0}, \pmb {\mathcal {Q}} \delta (t - t ^ {\prime})),\tag{2.68}
$$

where we note that we have defined it using the local variable (between control points $\mathbfit { T } _ { k }$ and $\pmb { T } _ { k + 1 } )$ . The transition function for this SDE is simply $\Phi ( t , s ) = \mathbf { I }$ and so stochastically integrating we have

$$
\boldsymbol {\xi} _ {k} (t) = \underbrace {\boldsymbol {\xi} _ {k} (t _ {k})} _ {\mathbf {0}} + \int_ {t _ {k}} ^ {t} \boldsymbol {w} (s) d s\tag{2.69}
$$

and then after taking the mean and covariance we can say that the motion prior is

$$
\boldsymbol {\xi} _ {k} (t) \sim \mathcal {G P} (\mathbf {0}, \min (t, t ^ {\prime}) \boldsymbol {\mathcal {Q}}).\tag{2.70}
$$

If we place our control-point poses uniformly spaced every $T$ seconds then our inverse kernel matrix will be simply $\kappa ^ { - 1 } = \Phi ^ { - \top } Q ^ { - 1 } \Phi ^ { - 1 }$ with

$$
\boldsymbol {\Phi} ^ {- 1} = \left[ \begin{array}{c c c c} \mathbf {I} & & & \\ - \mathbf {I} & \mathbf {I} & & \\ & \ddots & \ddots & \\ & & - \mathbf {I} & \mathbf {I} \end{array} \right], \quad \boldsymbol {Q} = \operatorname{diag} \left(\boldsymbol {\mathcal {K}} (t _ {1}, t _ {1}), T \boldsymbol {\mathcal {Q}}, \ldots , T \boldsymbol {\mathcal {Q}}\right).\tag{2.71}
$$

The individual errors in terms of the local variables will be

$$
\boldsymbol {e} _ {k} = \left\{ \begin{array}{l l} \mathrm{Log} (\boldsymbol {T} _ {1} \check {\boldsymbol {T}} _ {1} ^ {- 1}) & k = 1 \\ \boldsymbol {\xi} _ {k - 1} (t _ {k}) - \boldsymbol {\xi} _ {k - 1} (t _ {k - 1}) & k > 1 \end{array} \right.,\tag{2.72}
$$

where $\check { \pmb { T } } _ { 1 }$ is some prior initial pose value. In terms of the global variables, these same errors are

$$
\boldsymbol {e} _ {k} = \left\{ \begin{array}{l l} \operatorname{Log} \left(\boldsymbol {T} _ {1} \check {\boldsymbol {T}} _ {1} ^ {- 1}\right) & k = 1 \\ \operatorname{Log} \left(\boldsymbol {T} _ {k} \boldsymbol {T} _ {k - 1} ^ {- 1}\right) & k > 1 \end{array} \right..\tag{2.73}
$$

Figure 2.6 shows what the ‘random walk’ GP motion prior looks like as a factor graph. Similarly to the previous section discussing linear splines, if we want to query the trajectory at other times of interest, we can do this using GP interpolation. For the ‘random walk’ prior, this results again in linear interpolation [47]:

$$
\pmb {T} (t) = \left(\pmb {T} _ {k} \pmb {T} _ {k - 1} ^ {- 1}\right) ^ {\alpha_ {k} (t)} \pmb {T} _ {k - 1},\tag{2.74}
$$

where $\begin{array} { r } { \alpha _ { k } ( t ) = \frac { t - ( k - 1 ) T } { T } } \end{array}$ and $( k - 1 ) T \leq t < k T$ . In contrast to the spline method, this linear interpolation results indirectly from our choice of SDE at the beginning rather than an explicit choice. Choosing higher-order SDEs at the start will result in higher-order splines for interpolation.

The last part we need to understand is how to linearize our error terms for use in MAP estimation. To do this, we again make use of the Lie group perturbation approach detailed earlier. For example, looking at the second case in (2.73) we can write

$$
\begin{array}{c} \boldsymbol {e} _ {k} = \operatorname{Log} \left(\operatorname{Exp} (\boldsymbol {\xi} _ {k}) \boldsymbol {T} _ {k} ^ {0} \boldsymbol {T} _ {k - 1} ^ {0 ^ {- 1}} \operatorname{Exp} (- \boldsymbol {\xi} _ {k - 1})\right) \\ \approx \operatorname{Log} \left(\boldsymbol {T} _ {k} ^ {0} \boldsymbol {T} _ {k - 1} ^ {0 ^ {- 1}}\right) + \boldsymbol {\xi} _ {k} - \operatorname{Ad} \left(\boldsymbol {T} _ {k} ^ {0} \boldsymbol {T} _ {k - 1} ^ {0 ^ {- 1}}\right) \boldsymbol {\xi} _ {k - 1}, \end{array}\tag{2.75}
$$

where $\pmb { T } _ { k } ^ { 0 }$ and $\pmb { T } _ { k - 1 } ^ { 0 }$ are current guesses, $\xi _ { k }$ and $\xi _ { k - 1 }$ are the to-be-solved-for perturbations, and $\operatorname { A d } ( \cdot )$ is the adjoint for $\operatorname { S E } ( d )$ . This linearized form for $e _ { k }$ can be inserted in our standard MAP estimation framework at each iteration.

Additionally, if we want to use (2.74) to reduce the number of control points in this ‘random walk’ example, we can make use of the same approach developed for linear splines detailed in (2.67), since both methods boil down to linear interpolation between $\operatorname { S E } ( d )$ control points. Ultimately, then, the big diference between the spline and GP approaches is that the GP approach employs motion-prior terms (see Figure 2.6) to regularize the problem, while the spline approach does not.<sup>4</sup>

## 2.3 Further Readings & Recent Trends

Much has been written about carrying out estimation on manifolds and more specifically Lie groups. The seminal reference in robotics is Chirikjian [205] (see also Chirikjian and Kyatkin [207], Chirikjian [206]; these books are well grounded in theory and describe quite general tools for handling uncertainty, even globally, on manifolds. For more details on manifold optimization, Boumal [106] is an excellent reference with an eye towards robotics applications. Barfoot [47] ofers more details on handling estimation on Lie groups when the uncertainty is still somewhat compact, similar to what we have discussed in this chapter. For continuous-time estimation, Talbot et al. [1067] provides a comprehensive survey of both parametric and nonparametric methods; Barfoot [47] also discusses the nonparametric methods in some detail.

Similarly to the previous chapter, many of the current trends including handling outliers, making algorithms diferentiable, and using the tools presented herein to support diferent types of sensors will be discussed in the following chapters, so keep reading.

# Robustness to Incorrect Data Association and Outliers

Heng Yang, Josh Mangelson, Yun Chang, Jingnan Shi, Niko Sunderhauf, and Luca Carlone

In Chapter 1, we have seen that factor graphs are a powerful representation to model and visualize SLAM problems, and that maximum a posteriori (MAP) estimation provides a grounded and general framework to infer variables of interest $( e . g .$ , robot poses and landmark positions) given a set of measurements $( e . g .$ , odometry and landmark measurements). For instance, we observed that when the measurements $z _ { i }$ are afected by additive and zero-mean Gaussian noise with covariance $\Sigma _ { i }$ , MAP estimation leads to a nonlinear least-squares optimization:

$$
\boldsymbol {x} ^ {\mathrm{MAP}} = \underset {\boldsymbol {x}} {\arg \min} \sum_ {i} \| \boldsymbol {z} _ {i} - \boldsymbol {h} _ {i} (\boldsymbol {x} _ {i}) \| _ {\boldsymbol {\Sigma} _ {i}} ^ {2},\tag{3.1}
$$

where $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { i } }$ denotes the subset of the states involved in measurement $i . ^ { 1 }$ In this chapter we notice that in practice many measurements $z _ { i } \mathrm { \longrightarrow p o s s i b l y }$ due to incorrect data association— may have large errors, which are far from following a zero-mean Gaussian (Section 3.1); these measurements typically induce large perturbations in the estimate $\scriptstyle \mathbf { x } ^ { \mathrm { M A P } }$ from eq. (3.1). Therefore, we discuss how to reject gross outliers in the SLAM front-end (Section 3.2) and then focus on how to increase robustness to remaining outliers in the SLAM back-end (Section 3.3). We close the chapter with a short review of recent trends and provide extra pointers to related work (Section 3.4).

## 3.1 What Causes Outliers and Why Are They a Problem?

This section argues that outliers are inevitable in most SLAM applications and that not handling them appropriately leads to grossly incorrect estimates.

## 3.1.1 Data Association and Outliers

To understand the cause of outlier measurements, let us consider two examples. First, consider a landmark-based SLAM problem, where we have to reconstruct the trajectory of the robot and the position of external landmarks from odometry measurements and relative observations of landmarks from certain robot poses. Assuming (as we did in Chapter 1) that the landmark measurements have zero-mean Gaussian noise leads to terms in the optimization in the form $\| z _ { i j } - h ( p _ { i } , \ell _ { j } ) \| _ { \Sigma } ^ { 2 }$ These terms model the fact that a given measurement $z _ { i j }$ is an observation of landmark $\ell _ { j }$ from pose $\mathbf { \nabla } p _ { i }$ up to Gaussian noise, where $h ( \cdot )$ is the function describing the type of relative measurement $( e . g .$ , range, bearing, etc.). In practice, the measurements $z _ { i j }$ are obtained by pre-processing raw sensor data in the SLAM front-end. For instance, if the robot has an onboard camera and $z _ { i j }$ is a visual observation of the bearing to a landmark $\ell _ { j }$ , the measurement $z _ { i j }$ might be extracted by performing object (or more generally, feature) detection and matching in the image, and then computing the bearing corresponding to the detected pixels. Now, the issue is that the detections are imperfect and a landmark detected as $\ell _ { j }$ in the image, might be actually a diferent landmark in reality. This causes $z _ { i j }$ to largely deviate from the assumed model. The problem of associating a measurement to a certain landmark is typically referred to as the data association problem and is common to many other estimation problems (e.g., target tracking). Therefore, incorrect data association creates outliers in the estimation problem.

As a second example, consider a pose-graph optimization problem, where we are primarily interested in estimating the trajectory of the robot (represented as a set of poses), and the measurements are either odometry measurements (which relate consecutive poses along the trajectory) or loop closures (which relate nonconsecutive and possibly temporally distant poses). In practice, the loop closures are detected using $( e . g .$ , vision-based or lidar-based) place recognition methods, which are in charge of detecting if a pair of poses $\pmb { p } _ { i }$ and $p _ { j }$ have observed the same portion of the environment. Unfortunately current place recognition methods are prone to making mistakes and detecting loop closures between poses that are not observing the same scene. This is partially due to limitations of current methods, but it is often due to perceptual aliasing, that is the situation where two similarly looking locations actually correspond to diferent locations (think of two classrooms in a university building, or similarly looking cubicles in an ofice environment). This can be again understood as a failure of data association, where we mistakenly associate the loop closure measurement to two incorrectly chosen robot poses.

Note that outliers are not only caused by incorrect data association, but can also be caused by violations of the assumptions made in the SLAM approach. For instance, the majority of SLAM approaches assume landmarks to be static, hence detections of a moving object —even when correctly associated to that object— may lead to outlier measurements with large residuals. Similarly, sensor failure and degradation, $e . g .$ , a faulty wheel encoder or dust on the camera lens, might contribute to creating outliers in the measurements.

![](images/ddc222b702e516e02124c861c2f00219115135f9feacce05f4d6c64dcf3a872f.jpg)

![](images/3ccb49c7a253eb43cabf4b89fdc57cf7a6b1778cd2652068a9406a35d9b29a0a.jpg)

## 3.1.2 Least-Squares in the Presence of Outliers

In the presence of outliers, the estimate resulting from the least-squares formulation (3.1) can be grossly incorrect. From the theoretical standpoint, the Gaussian noise we assumed for the measurement is “light-tailed”, in that it essentially rules out the possibility of measurements with very large error. From a more practical perspective, the outliers lead to terms in the objective function where the residual error $r _ { i } ( \pmb { x } ) : = \| z _ { i j } - h ( \pmb { p } _ { i } , \pmb { \ell } _ { j } ) \| _ { \Sigma }$ is very large, when evaluated near the ground truth. Since the residuals are squared in the objective of the optimization, i.e., the objective is $\textstyle \sum _ { i } r _ { i } ( { \pmb x } ) ^ { 2 }$ , these residuals have a disproportionately large impact on the cost, and the optimization focuses on minimizing the large terms induced by the outliers rather than making good use of the remaining (inlier) measurements.

![](images/dd88ae612d25550517be2aa6d8ae6d17b3901bc1d6abdd4f5d62ccc83a7c1816.jpg)

![](images/14f55ee14950aebd1bd5a280ff3b53899d5efdb8ba1708bc444501ae9c30ac65.jpg)

![](images/46bebd1e4816f8ebacbb6a7544412b105d02a6f203ddb6a37bf18559bce2a3a8.jpg)

![](images/7ffa9ac1ea7a9168e51d46d96a15361d33da886fd43be4b1d60e1058b983f8e0.jpg)  
Figure 3.1 SLAM problems with outliers: (a)-(c) Ground truth trajectories for the M3500, SubT, and Victoria Park datasets. (d)-(f) Trajectory estimates obtained with the leastsquares formulation in the presence of outliers. Inlier measurements are visualized as gray edges, while outliers are visualized as red edges. In the SubT dataset, we also visualize a dense map built from the SLAM pose estimate.

To illustrate this point, Figure 3.1 shows results for three SLAM problems with outliers. The first column is a simulated pose-graph optimization benchmark, known as M3500, with poses arranged in a grid-like configuration; the dataset includes 3500 2D poses and 8953 measurements. The second column is a real-world pose-graph dataset, denoted as SubT, collected in a tunnel during the DARPA Subterranean Challenge [299]; the dataset includes 682 3D poses and 3278 measurements. The third column is a real-world landmark-SLAM dataset, known as Victoria Park [809]; the dataset includes 7120 2D poses and landmarks, and 17728 measurements. Figure 3.1(a)-(c) show the ground truth trajectories for the three problems. Figure 3.1 (d)-(f) show the estimate produced by the least-squares formulation in the presence of outliers. In particular, for M3500 and Victoria Park we add 15% random outliers to the (loop closures or landmark) measurements, while the SubT dataset already includes outliers. In the figure, we visualize outlier measurements in red. We observe that the presence of outliers leads to completely incorrect trajectories and map estimates. Moreover, the outliers often expose perceptual aliasing in the environment: for instance, the two similarly looking vertical corridors in the middle of the SubT dataset induce many spurious loop closures, which mislead the back-end to create a map with a single vertical corridor.

![](images/01acf9ddaa14e194a185fa6e53e754763021e5e62566968febee2dbd36f5084c.jpg)  
Figure 3.2 Feature tracking across three frames (collected at time $k - 2 , k - 1$ , and k) in a visual SLAM problem. Inliers are visualized in green: these are pixels picturing the same (static) 3D point over time. Outliers are visualized in red.

## 3.2 Detecting and Rejecting Outliers in the SLAM Front-end

The main role of the SLAM front-end is to extract intermediate representations or (pseudo-)measurements —which will be converted into factors for the backend— from the raw sensor data. Typical SLAM front-ends accomplish this by first computing an initial set of measurements (possibly corrupted by many outliers) and then post-processing the initial set to remove outliers. This section discusses two approaches to reject outliers in the SLAM front-end: RANSAC and Pairwise Consistency Maximization.

## 3.2.1 RANdom SAmple Consensus (RANSAC)

RANSAC is a well-established tool for outlier rejection [328] and is a key component of many landmark-based SLAM systems. In order to understand what RANSAC is and its role in SLAM, consider a landmark-based visual SLAM approach.

Example 3.1 (Outliers in landmark-based visual SLAM) A landmark-based (or feature-based) visual SLAM approach extracts 2D feature points in each image and then associates them across consecutive frames using either optical-flow-based feature tracking or descriptor-based feature matching (Figure 3.2). In particular, at time k, the approach detects 2D feature points and matches them with corresponding points observed in the previous frame (say, at time $k - 1 )$ ; the matching pixels are typically referred to as 2D-2D correspondences. Due to inaccuracies of optical flow or descriptor-based matching, this initial set of correspondences might contain outliers. Therefore, it is important to filter out gross outliers before passing them to the back-end, which estimates the robot poses and landmark positions.

RANSAC is a tool to quickly detect and remove outliers in the correspondences before passing them to the back-end. Detecting outliers relies on two key insights. The first insight is that in SLAM problems, inlier correspondences must satisfy geometric constraints. For instance, in our example, inlier correspondences picture the observed pixel motion of static 3D points as the camera moves. The resulting pixel motion cannot be arbitrary, but must follow a precise geometric constraint, known as the epipolar constraint, which dictates how corresponding pixels in two frames are related depending on the camera motion. In particular, for calibrated cameras, the epipolar constraint imposes that corresponding pixels $z _ { i } ( k - 1 ) , z _ { i } ( k )$ —picturing landmark i at time $k - 1$ and $k ,$ respectively— satisfy

$$
\boldsymbol {z} _ {i} (k - 1) ^ {\top} \left([ \boldsymbol {t} _ {k} ^ {k - 1} ] _ {\times} \boldsymbol {R} _ {k} ^ {k - 1}\right) \boldsymbol {z} _ {i} (k) = 0,\tag{3.2}
$$

where $t _ { k } ^ { k - 1 }$ and $\pmb { R } _ { k } ^ { k - 1 }$ are the relative position and rotation describing the (unknown) motion of the camera between time $k - 1$ and $k . ^ { 2 }$ More generally, if we denote the i-th correspondence as $z _ { i }$ (in the example above, $z _ { i } = \{ z _ { i } ( k - 1 ) , z _ { i } ( k ) \} )$ • these geometric constraints are in the form

$$
C (\boldsymbol {z} _ {i}, \boldsymbol {x}) \leq \gamma ,\tag{3.3}
$$

which states that the correspondences have to satisfy some inequality, which is possibly a function of the unknown state ${ \mathbf { \psi } } _ { \mathbf { { \psi } } }$ in (3.3) the parameter γ on the righthand-side is typically tuned to account for the presence of noise. For instance, while ideally the epipolar constraint in (3.2) is exactly satisfied, in practice it might have small errors since the pixel detections are noisy, and hence we would relax the constraint to only require $\begin{array} { r } { | z _ { i } ( k - 1 ) ^ { \mathsf { T } } \left( [ t _ { k } ^ { k - 1 } ] _ { \times } R _ { k } ^ { k - 1 } \right) z _ { i } ( k ) \vert \leq \gamma } \end{array}$ , for some small γ. The socond insight is that outliors

The second insight is that —assuming we do not have too many outliers— we can find the inliers as the largest set of correspondences that satisfy the geometric

constraint (3.3) for some x:

$$
\begin{array}{c} \mathsf {S} _ {\mathrm{CM}} ^ {*} = \underset {\boldsymbol {x}, \mathsf {S} \subset \mathsf {M}} {\operatorname{argmax}}   | \mathsf {S} | \\ \text { s.t. } C (\boldsymbol {z} _ {i}, \boldsymbol {x}) \leq \gamma , \quad \forall i \in \mathsf {S} \end{array}\tag{3.4}
$$

where M is the set of initial putative correspondences, and S denotes the cardinality (number of elements) in the subset S [728]. In words, the optimization (3.4) looks for the largest subset S of the set of putative correspondences M, such that measurements in S satisfy the geometric constraints for the same value of x. Intuitively, problem (3.4) captures the intuition that the inliers (estimated by the set S) must “agree” on the same x (e.g., they must all be consistent with the actual motion of the robot). Problem (3.4) is known as consensus maximization in computer vision. Note that (3.4) does not require solving the entire SLAM problem (which might involve many poses and landmarks), since it only involves a small portion of the SLAM state; for instance, the epipolar constraint (3.2) only involves the relative pose between two frames rather than the entire SLAM trajectory. At the same time, (3.4) is still a hard combinatorial problem, which clashes with the fast runtime requirements of typical SLAM front-ends. Therefore, rather than looking for exact solutions to (3.4), it is common to resort to quick heuristics to approximately solve (3.4).

RANdom SAmple Consensus (RANSAC) is probably the most well-known approach to find an approximate solution to the consensus maximization problem in (3.4). RANSAC builds on the key assumption that x in (3.4) is relatively lowdimensional and can be estimated from a small set of measurements (the so-called minimal set), using fast estimators (the so called minimal solvers).<sup>3</sup> For instance, in our visual SLAM example, one can estimate the relative motion between two camera frames using only 5 pixel correspondences, using Nister’s 5-point method [810]. Then the key idea behind RANSAC is that, instead of exhaustively checking every possible subset $\mathsf { S } \subset \mathsf { M }$ , one can sample minimal sets of measurements looking for inliers. More in detail, RANSAC iterates the following three steps:

1 Sample a subset of n correspondences, where n is the size of the minimal set for the problem at hand;<sup>4</sup>

2 Compute an estimate xˆ from the n sampled correspondences using a minimal solver;<sup>5</sup>

3 Select the correspondences ${ \mathsf { S } } \subset { \mathsf { M } }$ that satisfy the geometric constraint $C ( z _ { i } , \hat { \pmb x } ) \le$ γ for the xˆ computed at the previous step. Store the set S if it is larger than the set computed at the previous iterations.

The set S computed in the last step is called the consensus set and RANSAC typically stops after computing a suficiently large consensus set (as specified by a user parameter) or after a maximum number of iterations. RANSAC essentially attempts to sample n inliers from the set of measurements, since these are likely to “agree” with all the other inliers and hence have a large consensus set.

RANSAC is the go-to solution for many outlier-rejection problems. In particular, it quickly converges to good estimates (i.e., good sets of correspondences) in problems with small number of outliers and small minimal sets. Assuming that the probability of sampling an inlier from the set of measurements is $\omega , ^ { 6 }$ it is easy to conclude that the expected number of iterations RANSAC requires for finding a set of inliers is $\scriptstyle { \frac { 1 } { \omega ^ { n } } }$ . For instance, when $n = 5$ and $\omega = 0 . 7$ (i.e., 70% of the measurements are inliers), the expected number of iterations is less then 10. This, combined with the fact that non-minimal solvers are extremely fast in practice (often allowing hundreds of iterations in a handful of milliseconds), makes RANSAC extremely appealing. Moreover, RANSAC also provides an estimate xˆ $( e . g .$ , the robot odometry), that can be useful as an initial guess for the back-end.

On the downside, RANSAC may not be the right approach for all problems. In particular, the expected number of iterations becomes impractically large when the number of inliers is small or when the minimal set is large; for instance, when $n = 1 0$ and $\omega = 0 . 1$ , the expected number of iterations to find a set of inliers becomes $1 0 ^ { 1 0 }$ and terminating RANSAC after a smaller number of iterations is likely to return incorrect solutions $( i . e . ,$ , incorrect xˆ and correspondences). As we discuss in the next section, in the context of many SLAM problems, the assumptions of having many inliers and small minimal sets are not always valid.

## 3.2.2 Graph-theoretic Outlier Rejection and Pairwise Consistency Maximization

As we mentioned, RANSAC is very efective when the number of outliers is reasonable (say, below 70%) and the size of the minimal set is small (say, less than 8). However, environments with severe perceptual aliasing might have very high number of outliers. Moreover, not all the problems we are interested in have a fast minimal solver with a small minimal set. For instance, if we consider a pose-graph SLAM problem with N nodes, the minimal set must include at least $N - 1$ measurements (forming a spanning tree of the pose-graph), and N is typically in the thousands.

For these reasons, this section introduces an alternative approach, known as

Association and Outliers

![](images/84029a2147cbfeb042fa3a747daf672039fa3529a7e21e2ef318d1a92d5c4800.jpg)  
Figure 3.3 (a) 3D-3D correspondences from two RGB-D scans representing two partial views of a scene. The green lines indicate inlier correspondences and the red lines indicate outlier correspondences. (b) Pose graph with outliers in the loop closures. The dashed green lines indicate inlier loop closures while the dotted red line is an outlier loop closure.

Pairwise Consistency Maximization $( P C M )$ , that, rather than sampling minimal sets, seeks to find the largest set of measurements that are internally “consistent” with one another, using graph theory. This approach can be used to sort through sets of measurements with upwards of 90% outliers and prune gross outliers before passing them to the back-end. The approach was initially proposed in [728] and extended beyond pairwise consistency in [1005, 332].

The key insight behind PCM is that for many problems one can define consistency functions that capture whether a pair of measurements are consistent with each other. Let’s elucidate on this point with two examples.

Example 3.2 (Consistency Function in landmark-based visual SLAM with RGB-D cameras) A landmark-based visual SLAM approach with RGB-D cameras extracts 3D feature points in each RGB-D frame and then associates them across consecutive frames (Figure $3 . 3 ( \mathrm { a } ) \ r ,$ . In particular, at time k, the approach detects 3D feature points and matches them with corresponding points observed in the previous frame (say, at time $k - 1 )$ ; the matching 3D points are typically referred to as 3D-3D correspondences. We observe that the 3D points collected at time k and $k - 1$ ideally correspond to the same set of 3D static points observed from two diferent viewpoints; therefore, the distance between a pair of corresponding points $\{ z _ { i } ( k -$ $1 ) , z _ { j } ( k - 1 ) \}$ and $\{ z _ { i } ( k ) , z _ { j } ( k ) \}$ has to be constant over time (up to noise):

$$
| | | \boldsymbol {z} _ {i} (k - 1) - \boldsymbol {z} _ {j} (k - 1) | | - | | \boldsymbol {z} _ {i} (k) - \boldsymbol {z} _ {j} (k) | | | \leq \gamma\tag{3.5}
$$

We observe that contrary to the geometric constraints used in RANSAC, the consistency function (3.5) (i) does not depend on the state, hence it can be evaluated directly without the need for a minimal solver, and (ii) involves a pair of correspondences regardless of the size of the minimal set. While the previous example could also be solved with $\mathrm { R A N S A C } , ^ { 7 }$ let us now consider a higher dimensional problem.

Example 3.3 (Consistency Function in pose-graph SLAM) Consider a posegraph SLAM problem where loop closures might contain outliers due to place recognition failure and perceptual aliasing; we assume the odometry is reliable and outlier free. In order to understand if two loop closures are consistent with each other, we observe that in the noiseless case, pose measurements along cycles in the graph must compose to the identity (Figure $3 . 3 ( \mathrm { b } ) ) . ^ { 8 }$ Therefore, a pair of loop closures $\pmb { T _ { b } ^ { a } }$ (between poses a and b) and $\pmb { T } _ { d } ^ { c }$ (between poses c and $d )$ must satisfy:

$$
\mathrm{dist} (\pmb {T} _ {b} ^ {a} \cdot \bar {\pmb {T}} _ {c} ^ {b} \cdot \pmb {T} _ {d} ^ {c} \cdot \bar {\pmb {T}} _ {a} ^ {d}, \mathbf {I}) \leq \gamma\tag{3.6}
$$

where $\hat { \mathbf { T } } _ { c } ^ { b }$ and $\bar { \pmb { T } } _ { a } ^ { d }$ are the chain of odometry measurements from node b to node $c ,$ and from node d to node a, respectively, and dist is a suitable distance function that measures how far is $\pmb { T } _ { b } ^ { a } \cdot \bar { \pmb { T } } _ { c } ^ { b } \cdot \pmb { T } _ { d } ^ { c } \cdot \bar { \pmb { T } } _ { a } ^ { d }$ from the identity pose. As usual, $\gamma$ is a parameter chosen to account for the noise: measurements along a loop might not compose to the identity due to noise in the odometry and loop closures.<sup>9</sup>

More generally, a consistency function is a function relating two measurements and that have to satisfy a given constraint. For a pair of measurements $z _ { i }$ and $z _ { j }$ , the resulting pairwise consistency constraints are in the form:

$$
F (\boldsymbol {z} _ {i}, \boldsymbol {z} _ {j}) \leq \gamma ,\tag{3.7}
$$

where $F$ is the consistency function, and $\gamma$ is a user-specified parameter that accounts for measurement noise. We remark that the pairwise consistency constraint are state independent, hence they can be eficiently checked without resorting to a minimal solver by just inspecting every pair of measurements.

Using (3.7), we can formulate an alternative approach for outlier rejection, which selects the largest set of measurements that are pairwise consistent:

$$
\begin{array}{c} \mathsf {S} _ {\mathrm{PCM}} ^ {*} = \underset {\mathsf {S} \subset \mathsf {M}} {\operatorname{argmax}} | \mathsf {S} | \\ \text {s.t.} F (\boldsymbol {z} _ {i}, \boldsymbol {z} _ {j}) \leq \gamma , \quad \forall i, j \in \mathsf {S} \end{array}\tag{3.8}
$$

Problem (3.8) looks for the largest subset S of measurements such that every pair of measurements in S are pairwise consistent. We refer to this as the pairwise consistency maximization (PCM) problem. This problem is still combinatorial in nature, but appears slightly easier than (3.4): the problem does not involve x, and the constraints $F ( z _ { i } , z _ { j } ) \leq \gamma$ can be pre-computed for every pair $( i , j )$ in M. Furthermore, the problem admits a graph-theoretic interpretation, which allows solving (3.4) using well-established tools from graph theory, namely, maximum clique algorithms.

In order to draw a connection between problem (3.8) and graph theory, let us visualize the outlier-rejection problem as a graph ${ \mathcal { G } } ,$ where the nodes of the graph are the putative measurements $i \in { \mathsf { M } }$ and an edge exists between two nodes i and $j$ if

![](images/f405223ca19647a424ddeb6d4d8375573b5020f5b580ad455f92e0a6a438e986.jpg)  
Figure 3.4 (a) Consistency graph of the 3D-3D correspondences example in Figure 3.3(a). (b) Consistency graph of the loop closures for the pose-graph example in Figure 3.3(b).

$F ( z _ { i } , z _ { j } ) \leq \gamma$ . This is typically called the consistency graph. Now problem (3.8) asks to select the largest subset of nodes S such that every pair of nodes in S is connected by an edge: this is exactly the definition of maximum clique of a graph. More in detail, a clique in graph theory is a subset of nodes in which every pair of nodes has an edge between them, and the maximum clique is the largest such subset of nodes in the graph. Therefore, the solution to problem (3.8) is the maximum clique of the consistency graph . This graph theoretic connection is really useful in practice, since the problem of finding the maximum clique for a given graph is a well-studied problem in graph theory and is called the maximum clique problem. The maximum clique problem is an NP-hard problem [1197] and hard to approximate [1314, 322], meaning that finding a solution arbitrarily close to the true solution is also NPhard. However, dozens of potential maximum clique algorithms have been proposed, some of which can handle significantly sized problems depending on the density of the graph. The majority of proposed methods can be classified as either exact or heuristic-based methods. All of the exact algorithms are exponential in complexity and are usually based on branch and bound, while the heuristic algorithms often try to exploit some type of structure in the problem, making them faster, at the expense of not necessarily guaranteeing the optimal solution [1197]. Relatively recent works, e.g., [853], propose maximum clique algorithms that are parallelizable and able to quickly find maximum cliques in large sparse graphs.

In summary, solving the PCM problem using a maximum clique algorithm involves the following steps:

1 Select a Consistency Function F for the problem at hand.

2 Evaluate the Consistency Function for every pair of putative measurements $( i , j ) \in$ M, and create a consistency graph with edges between i and j when $F ( z _ { i } , z _ { j } ) \leq \gamma$

3 Solve for the Maximum Clique of the Consistency Graph using exact or approximate maximum clique algorithms.

4 Return measurements S in the (possibly approximate) maximum clique.

We remark that the choice of consistency function is problem-dependent. Moreover, choosing a good consistency function might largely influence the quality of the outlier rejection. For instance, one could select a dummy function $F ( z _ { i } , z _ { j } ) = 0$ which always returns zero regardless of the arguments; such a function would not allow rejecting any outliers, hence making PCM inefective. On the other hand, if we make the function such that only the inliers can pass the test, then we would exactly reject all the outliers. A selection of potential consistency functions for a broad variety of geometric problems is discussed in [1005, 332].

Before concluding this section a few remarks are in order. While we observed that PCM has the ability to handle a large number of outliers compared to RANSAC and is more suitable for certain problems (e.g., pose-graph SLAM), the trade-ofs between PCM and RANSAC are more nuanced. RANSAC evaluates the consistency of individual measurements using an estimate computed by a minimal solver; PCM, on the other hand, evaluates the consistency of a set of measurements to each other in a pairwise manner. In certain cases, RANSAC’s individual consistency is insuficient to evaluate the set of measurements as a whole: this is often the case in pose-graph optimization where individual consistency of a pair of loop closure measurements does not necessarily ensure pairwise consistency of the two loop closures.<sup>10</sup> On the other hand, for certain problems such as 3D-3D pose estimation (Example 3.2), the pairwise consistency function (3.5) used in PCM might be more permissive then RANSAC and lead to classifying certain outliers as inliers.

In the context of PCM, it is also important to note that exact maximum clique solvers tend to be slow in dense consistency graphs (i.e., when many pairs of measurements are consistent), hence heuristic-based maximum-clique solutions may be a better choice for certain problems. Finally, for certain problems, it might be hard to design a suitable consistency function; for instance, for 2D-2D correspondences, there is no easy way to rigorously design a general consistency function due to the lack of suitable invariances (see discussion in [1005]).

## 3.3 Increasing Robustness to Outliers in the SLAM Back-end

Front-end outlier rejection, including both RANSAC and PCM, might still miss outliers and pass an outlier-contaminated set of measurements to the back-end.<sup>11</sup> As we have seen in Section 3.1.2, a handful of outliers can lead to completely wrong results when using standard least squares estimation. Therefore, it is important to enhance the back-end to be robust to remaining outliers.

In Section 3.1.2, we observed that the use of squared residuals “amplifies” the impact of outlying measurements on the cost function. In this section we slightly modify the objective function in the SLAM optimization to regain robustness to outliers, following the standard theory of M-Estimation in robust statistics [495].

M-Estimation (“Maximum-likelihood-type Estimation”) is a framework for robust estimation and suggests replacing the squared loss in eq. (3.1) with a suitably chosen robust loss function $\rho \colon$

$$
\boldsymbol {x} ^ {\text { MAP }} = \underset {\boldsymbol {x}} {\arg \min} \sum_ {i} \boldsymbol {r} _ {i} (\boldsymbol {x}) ^ {2} \quad \Longrightarrow \quad \boldsymbol {x} ^ {\text { MEST }} = \underset {\boldsymbol {x}} {\arg \min} \sum_ {i} \rho \left(\boldsymbol {r} _ {i} (\boldsymbol {x})\right).\tag{3.9}
$$

The key requirement for the robust loss $\rho$ is to be a non-negative function and grow less than quadratically for large residuals; in other words, robust loss functions need to have derivative $\begin{array} { r } { \frac { \partial \rho ( \pmb { r } _ { i } ) } { \partial \pmb { r } _ { i } } \ll \frac { \partial \| \pmb { r } _ { i } \| ^ { 2 } } { \partial \pmb { r } _ { i } } = 2 \pmb { r } _ { i } } \end{array}$ as $\mathbf { \nabla } r _ { i }$ becomes large; in many cases, it is desirable for $\frac { \partial \rho ( \pmb { r } _ { i } ) } { \partial \pmb { r } _ { i } }$ to approach zero as $\mathbf { \nabla } _ { \mathbf { r } _ { i } }$ becomes large. To elucidate this requirement, consider the case where we solve min $_ { \mathbf { \mathcal { x } } } \sum _ { i } \rho \left( \pmb { r } _ { i } ( \pmb { x } ) \right)$ using gradient descent. Using the chain rule, the gradient of the objective $\begin{array} { r } { f ( \pmb { x } ) \doteq \sum _ { i } \rho \left( \pmb { r } _ { i } ( \pmb { x } ) \right) } \end{array}$ becomes:

$$
\frac {\partial f}{\partial \boldsymbol {x}} = \sum_ {i} \frac {\partial \rho (\boldsymbol {r} _ {i})}{\partial \boldsymbol {r} _ {i}} \cdot \frac {\partial \boldsymbol {r} _ {i} (\boldsymbol {x})}{\partial \boldsymbol {x}}\tag{3.10}
$$

From (3.10), it is clear that if we start for a good initial guess $( i . e . ,$ relatively close to the ground truth), outlier measurements will have large residual and hence very small $\frac { \breve { \partial } \rho ( \pmb { r } _ { i } ) } { \partial \pmb { r } _ { i } }$ , thus having a minor influence on the overall descent direction. Hence they will have almost no influence in the estimation. Indeed, the function $\begin{array} { r } { \psi ( \pmb { r } _ { i } ) : = \frac { \partial \hat { \rho } ( \pmb { r } _ { i } ) } { \partial \pmb { r } _ { i } } } \end{array}$ is typically referred to as the influence function [85].

Rather than a single choice of robust loss function, the robust estimation literature provides a “menu” of potential choices. Figure 3.5 lists common choices of loss functions. This list includes common robust losses, such as Huber, Geman-McClure, Tukey’s biweight and the truncated quadratic loss, and also includes a more radical choice, named maximum consensus loss. The latter is not typically listed among the loss functions in the robust estimation literature, but we mention it here, since it connects back to the consensus maximization problem we discussed in $( 3 . 4 ) . ^ { 1 2 }$ The choice of robust loss is fairly problem-dependent [554, 1075]. For instance, loss functions with hard cut-ofs $( e . g .$ , the truncated quadratic loss, where there is a sudden transition between the quadratic and the “flat” portion of the function) are preferable when a reasonable threshold for the cut-of (i.e., the maximum error expected from the inliers) is known. One also has to take into account computational considerations. For instance, Huber is often used in BA problems since it is a convex function and is better-behaved during the optimization, despite leaving non-zero influence for the outliers (the influence becomes zero only if the

![](images/d62f0b14a781770de0d928a2bc5db48feb73bad0b5cecf4fe695e8f16d77a901.jpg)  
(a) Quadratic loss

![](images/c57bf228a0cde1e02ee2cc1c508ef0c788bff27c89ec49502de7fb1caba5964f.jpg)  
(b) Huber loss

![](images/abd3380a85d6de0464cb838eddd7048f4f923ca178f7f16256765e143810ad3b.jpg)  
(c) Geman-McClure

![](images/f41954a5877f37fd3f5089ab6964361b69b75d4b3ec9ee80360e1506a2277964.jpg)  
(d) Tukey’s biweight

![](images/f458be9aab9f793c517fb2748ea55d4d143c45f62830ce2e66af3492680a7929.jpg)

![](images/0ba2757187eef46948d72a72d4db2b40f92e9dab77d541d37f5f9517c1f5d87c.jpg)  
(e) Truncated Quadratic  
(f) Maximum Consensus loss

Figure 3.5 Quadratic loss and examples of robust loss functions. The shape of the robust loss functions is controlled by a parameter that controls the separation between inliers and outliers.

loss is constant for large residuals). On the other hand, the truncated quadratic and maximum consensus losses are known to be particularly insensitive to outliers, but they often require ad-hoc solvers.<sup>13</sup>

Figure 3.6(g)-(l) show the SLAM trajectories obtained by applying gradient descent to two of the robust losses mentioned above: the Huber loss and the truncated quadratic loss. Here we consider the same datasets used in Figure 3.1. We implemented the gradient descent solver using GTSAM’s NonlinearConjugateGradientOptimizer [253] with the gradientDescent flag enabled, and using robust noise models to instantiate the robust loss functions. We set the maximum number of iterations and the stopping conditions thresholds (relative and absolute tolerance) to 10000 and $1 0 ^ { - 7 }$ , respectively. All other parameters were left to the default GT-SAM values. In the figure, an edge is colored in gray if it is correctly classified as inlier or outlier by the optimization (i.e., an inlier that falls in the quadratic region of the Huber or truncated quadratic loss); it is colored in red if it is an outlier incorrectly classified as an inlier (a “false positive”); it is colored in blue if it is an inlier incorrectly classified as an outlier (a “false negative”). Compared to (non-robust) least squares optimization (Figure 3.1(d)-(f)), we note that the use of the robust losses allows us to quickly regain robustness to outliers in the case of the M3500 and SubT datasets, but a simple gradient descent method may still fail to correctly optimize heavily non-convex functions such as the truncated quadratic cost, as in the case of the Victoria Park dataset. We will address this issue with a better solver, based on graduated non-convexity, below. Moreover, while gradient descent already improves performance in many of the instances, as shown in Figure 3.6, it has slow convergence tails. For instance, in our experiments, it often takes thousands of iterations to converge. Therefore, in the rest of this section we discuss more advanced solvers, that improve both convergence quality and speed.

M3500  
![](images/94b8f7648085474b55ce2dca350bb3e76fd2834866375f7f01852ee6305aaca1.jpg)  
(a)  
SubT

![](images/895b8b8f94a3ce2bb52899e3677c80b447412234411ba1eb8645019aa617a9cb.jpg)  
(b)  
Victoria Park

![](images/0fe34b30ed9dd12ac0a191a3c98a1553402d66ff07694f212c4847e7fd0ffb11.jpg)

![](images/740e4b3e024d267da11078510075d736709ab6c40b47a8a281810f6c36e90088.jpg)  
(g)

(c)  
![](images/e495738bc6f276a9f347519cec47034b22a623e4cf349d459d1ad90c61e1d976.jpg)

![](images/d60fedf39af3e05f868b32152c41a5f32aac657c9804c48482bdb46bb5c314bd.jpg)

![](images/85c8e422b6bfba361c12de6e5d4625705cbad67c27e1288e7d585a3831ea9605.jpg)  
(j)

(h)  
(i)  
![](images/3d959690ff503efd6feab0e359665c47153cc67e22401e00bf11f8da991c418c.jpg)  
(k)

![](images/315a9c1c1ab8526b9fb06c328de349f4eeccd799161aa0d486bc8260e16faf35.jpg)  
(l)  
Figure 3.6 Solving SLAM problems with outliers using robust loss functions and gradient descent: (a)-(c) Ground truth trajectories for the M3500, SubT, and Victoria Park datasets. (d)-(f) Trajectory estimates obtained using gradient descent and Huber loss. (h)-(l) Trajectory estimates obtained using gradient descent and truncated quadratic loss. Measurements are visualized as colored edges. In particular, an edge is colored in gray if it is correctly classified as inlier or outlier by the optimization $( i . e . ,$ an inlier that falls in the quadratic region of the Huber or truncated quadratic loss); it is colored in red if it is an outlier incorrectly classified as an inlier (a “false positive”); it is colored in blue if it is an inlier incorrectly classified as an outlier (a “false negative”).

As a concluding remark before delving into more advanced solvers, we observe that while it might seem that we gave up on our probabilistic framework when switching to robust loss functions, it is actually possible to derive several robust losses by applying MAP estimation to heavy-tailed noise distributions. For instance, the truncated quadratic loss results from MAP estimation when assuming the noise to follow a max-mixture distribution between a Gaussian density (describing the inliers) and a uniform distribution (describing the outliers) [34].

## 3.3.1 Iteratively Reweighted Least Squares

M-Estimation replaces the least-squares loss by a robust loss $\rho$ in (3.9) — a function that grows sub-quadratically for large residuals. This comes with two prices. First, we lose the eficient solutions already developed for least-squares formulations; for instance, the Gauss-Newton and the Levenberg-Marquardt methods are designed for least squares problems. Second, due to the typical non-convex landscape of M-Estimation, iterative solvers $( e . g .$ , based on gradient descent) are sensitive to the quality of initialization and often converge to undesired suboptimal estimates. In this section, we introduce a popular algorithm for solving M-Estimation called iteratively reweighted least squares (IRLS) which, as the name suggests, allows reusing the eficient least-squares solvers. Then, in the next section, we introduce graduated non-convexity as a technique to improve the convergence of IRLS.

The basic idea behind IRLS is to optimize (3.9) by solving a weighted least squares problem at each iteration

$$
\boldsymbol {x} ^ {(t + 1)} = \underset {\boldsymbol {x}} {\arg \min} \sum_ {i} w _ {i} (\boldsymbol {x} ^ {(t)}) r _ {i} ^ {2} (\boldsymbol {x}),\tag{3.11}
$$

where the weights $w _ { i }$ ’s depend on the estimate $\mathbf { \boldsymbol { x } } ^ { ( t ) }$ from the last iteration. We wish the iterative solutions $\mathbf { \boldsymbol { x } } ^ { ( t ) }$ to converge to the optimal solution of M-Estimation (3.9). This implies that the gradient of the robust loss $\rho ,$ shown in (3.10), must match the gradient of the loss in (3.11). By writing down the gradient of (3.11) as

$$
\sum_ {i} 2 w _ {i} (\boldsymbol {x} ^ {(t)}) r _ {i} (\boldsymbol {x}) \frac {\partial r _ {i} (\boldsymbol {x})}{\partial \boldsymbol {x}}
$$

and comparing it to (3.10), we obtain the IRLS weight update rule

$$
w _ {i} (\pmb {x} ^ {(t)}) = \frac {1}{2 r _ {i} (\pmb {x} ^ {(t)})} \frac {\partial \rho (r _ {i} (\pmb {x} ^ {(t)}))}{\partial r _ {i} (\pmb {x} ^ {(t)})} = \frac {\psi (r _ {i} (\pmb {x} ^ {(t)}))}{2 r _ {i} (\pmb {x} ^ {(t)})},\tag{3.12}
$$

where we recall that $\begin{array} { r } { \psi ( r _ { i } ) : = \frac { \partial \rho ( r _ { i } ) } { \partial r _ { i } } } \end{array}$ is the influence function. Therefore, IRLS alternates computing the weights $w _ { i } ( \pmb { x } ^ { ( t ) } )$ for each measurement i, with performing an optimization step $( i . e . , \mathrm { ~ a ~ }$ Gauss-Newton or Levengberg-Marquardt iteration) on the weighted least squares problem (3.11).

Figure 3.7 shows the performance of IRLS on the M3500, SubT, and Victoria Park datasets. IRLS converges in tens of iterations and is typically much faster than gradient descent; for instance, gradient descent requires around 5 seconds to optimize the Huber loss in our M3500 experiments, while IRLS takes less than 1.5 seconds. On the other hand, this faster convergence often comes at the cost of a slightly decreased accuracy, as can be seen by comparing Figure 3.7 and Figure 3.6. The convergence properties of the update rule (3.12) has been studied in [10, 817].

## 3.3.2 Black-Rangarajan Duality

The weight update rule (3.12) is widely used in practice, but its derivation was somewhat heuristic. It also has the issues that (3.12) is not well-defined at the nondiferentiable points of $\rho \left( e . g . \right.$ ., the cut-of point of the truncated quadratic loss). We now introduce a more principled framework, namely the Black-Rangarajan $( B \ – R )$ duality [85], to solve M-Estimation using IRLS.

Let us present the intuition of B-R duality using the truncated quadratic loss

$$
\rho (r _ {i} (\boldsymbol {x})) := \min \{r _ {i} ^ {2} (\boldsymbol {x}), \beta_ {i} ^ {2} \},\tag{3.13}
$$

where $\beta _ { i } ^ { 2 }$ is a bound on the i-th residual such that the i-th measurement is an inlier if $r _ { i } ^ { 2 } ( { \pmb x } ) \leq \beta _ { i } ^ { 2 }$ and an outlier otherwise. We observe that the cost in (3.13) can be equivalently written as a sum of two terms by introducing a new weight variable $w _ { i } \in [ 0 , 1 ]$

$$
\rho (r _ {i} (\boldsymbol {x})) := \min _ {w _ {i} \in [ 0, 1 ]} w _ {i} r _ {i} ^ {2} (\boldsymbol {x}) + (1 - w _ {i}) \beta_ {i} ^ {2},\tag{3.14}
$$

where the first term is exactly the weighted least squares, and the second term is a function of $w _ { i }$ that does not depend on x. With (3.14), the M-Estimation problem (3.9) with a truncated quadratic loss can be reformulated as

$$
\min _ {\boldsymbol {w} _ {i} \in [ 0, 1 ], i = 1, \dots , N} \sum_ {i} \left[ w _ {i} r _ {i} ^ {2} (\boldsymbol {x}) + (1 - w _ {i}) \beta_ {i} ^ {2} \right],\tag{3.15}
$$

where we have introduced one $w _ { i }$ for each measurement residual $r _ { i } ( { \pmb x } )$ . Problem (3.15) is easy to interpret: $w _ { i } ~ = ~ 1$ implies $r _ { i } ^ { 2 } ( { \pmb x } ) ~ \le ~ \beta _ { i } ^ { 2 }$ and the i-th measurement is an inlier; $w _ { i } = 0$ implies $r _ { i } ^ { 2 } ( { \pmb x } ) > \beta _ { i } ^ { 2 }$ and the i-th measurement is an outlier. Moreover, all the residuals with $w _ { i } = 0$ are efectively discarded from the optimization (3.15) and hence robustness is ensured.

B-R duality generalizes the derivation above to a family of robust losses.

Theorem 3.4 (Black-Rangarajan Duality [85]) Given a robust loss function $\rho ( \cdot )$ , define $\phi ( z ) : = \rho ( \sqrt { z } )$ . If ϕ(z) satisfies lim<sub>z 0</sub> $\phi ^ { \prime } ( z ) = 1$ , lim $\iota _ { z \to \infty } \phi ^ { \prime } ( z ) = 0$ , and $\phi ^ { \prime \prime } ( z ) < 0$ , then the M-Estimation problem $( 3 . 9 )$ is equivalent to

$$
\min _ {\boldsymbol {x}} \sum_ {i = 1} ^ {N} \left[ w _ {i} r _ {i} ^ {2} (\boldsymbol {x}) + \Phi_ {\rho} (w _ {i}) \right],\tag{3.16}
$$

where $w _ { i } \in [ 0 , 1 ] , i = 1 , \ldots , N$ are weight variables associated to each residual $r _ { i } ,$ and the function $\Phi _ { \rho } ( w _ { i } )$ , referred to as an outlier process, defines a penalty on the weight $w _ { i }$ whose form is dependent on the choice of robust loss $\rho .$

![](images/1d130e48ce5ae89eb8ea8000207184c0f46a92d0179c5bd4cd972e0e41ef7401.jpg)  
(a)

![](images/22004b752ab47b2f04d0b7a15f4b9578f0daee5a9933cddbe2078e905e5b17f8.jpg)  
(b)

![](images/caa9b3945f4dbad245aac6253ee8d01d1bde3bc8927179457a11efc3014a54aa.jpg)  
(c)

![](images/efd597cb0e6be3d3c4e7ca6490473b66fb6a407d9a225458dc8e26b0cecc5a7e.jpg)  
(d)

![](images/28bb557ca0cd61c56726206a0b9c994e3f1b56c337e89bb31e91beddca632adb.jpg)  
(e)

![](images/fbbe5ebf8b01008f045c9ff6c693d5f1b34e80aabd42b74778dbf831b1ea693b.jpg)

![](images/38194e893d58a0b04f16405f4ebcd6be594f23bae01374191eecebd6b9c446ec.jpg)

![](images/df4331d7cfcda9d6a9b97f77d0a154cedc575e3ddcc79dc0e110c75705e22a05.jpg)

(f)  
![](images/4719d820b655c27cc77f2a77b460607f77aeb990430e4ad9caa17e68cbc36e75.jpg)

(g)  
![](images/47f5bc57cbecd5a73c7edad8f51de5a1c2a3faeb922a964508678a91230f8943.jpg)  
(j)

(h)  
![](images/6adeb320e916550af9d861e27c0b5cf00b8a383c510ede33cbb133457c855d17.jpg)  
(k)

(i)  
![](images/d395015818cefe925401bd4aecbd5f6b186796d121313b9dafefd02772df11f2.jpg)  
(l)  
Figure 3.7 Solving SLAM problems with outliers using robust loss functions and Iteratively Reweighted Least-Squares (IRLS): (a)-(c) Ground truth trajectories for the M3500, SubT, and Victoria Park datasets. (d)-(f) Trajectory estimates obtained using the Huber loss. $( \mathrm { g } ) \mathrm { - ( i ) }$ Trajectory estimates obtained using the Geman-McClure loss. $( \mathrm { j } ) \ u { - } ( \mathrm { l } )$ Trajectory estimates obtained using the truncated quadratic loss. Measurements are visualized as colored edges. In particular, an edge is colored in gray if it is correctly classified as inlier or outlier by the optimization (i.e., an inlier that falls in the quadratic region of the Huber or truncated quadratic loss); it is colored in red if it is an outlier incorrectly classified as an inlier (a “false $\mathrm { p o s i t i v e } ^ { , 9 } ) ;$ it is colored in blue if it is an inlier incorrectly classified as an outlier (a “false negative”).

In the case of $\rho$ being the truncated quadratic loss, we easily derived from (3.14) that $\Phi _ { \rho } ( w _ { i } ) = ( 1 - w _ { i } ) \beta _ { i } ^ { 2 }$ . When $\rho$ takes other forms, [85] provides a recipe to derive $\Phi _ { \rho } ( w _ { i } )$ . We give an example for the Geman-McClure (G-M) robust loss.

Example 3.5 (B-R Duality for G-M Loss) Consider the G-M robust loss function

$$
\rho (r _ {i} (\pmb {x})) = \frac {\beta_ {i} ^ {2} r _ {i} ^ {2} (\pmb {x})}{\beta_ {i} ^ {2} + r _ {i} ^ {2} (\pmb {x})},\tag{3.17}
$$

where $\beta _ { i } ^ { 2 }$ is a noise bound for the i-th residual similar to (3.13). The outlier process associated to (3.17) is

$$
\Phi_ {\rho} (w _ {i}) = \beta_ {i} ^ {2} (\sqrt {w _ {i}} - 1) ^ {2}.\tag{3.18}
$$

To verify the correctness of (3.18), consider

$$
\min _ {w _ {i} \in [ 0, 1 ]} w _ {i} r _ {i} ^ {2} (\boldsymbol {x}) + \Phi_ {\rho} (w _ {i}),\tag{3.19}
$$

whose optimal solution is (via setting the gradient of (3.19) to zero)

$$
w _ {i} ^ {\star} = \left(\frac {\beta_ {i} ^ {2}}{r _ {i} ^ {2} (\pmb {x}) + \beta_ {i} ^ {2}}\right) ^ {2}.\tag{3.20}
$$

Plugging (3.20) back to the objective of (3.19) recovers the G-M robust loss (3.17).

## 3.3.3 Alternating Minimization

With the introduction of B-R duality, the IRLS algorithm naturally comes out using a common optimization strategy called alternating minimization [1112, 80]. The idea is that, although it is dificult to jointly optimize both x and $w _ { i } \in [ 0 , 1 ] , \ i$ i = $1 , \ldots , N$ in (3.16), optimization of either x or $w _ { i } \mathrm { ^ s }$ when fixing the other is easy. To see this, observe that when $w _ { i } \mathrm { ^ s }$ are fixed, problem (3.16) becomes a weighted least squares; analogously, when x is fixed, problem (3.16) becomes

$$
\min _ {w _ {i} \in [ 0, 1 ], i = 1, \dots , N} \sum_ {i = 1} ^ {N} \Phi_ {\rho} (w _ {i}) + w _ {i} r _ {i} ^ {2} (\boldsymbol {x}),
$$

which splits into N subproblems, each optimizing a scalar $w _ { i }$

$$
\min _ {w _ {i} \in [ 0, 1 ]} \Phi_ {\rho} (w _ {i}) + w _ {i} r _ {i} ^ {2} (\boldsymbol {x}).\tag{3.21}
$$

Problem (3.21) is easy to solve and often admits a closed-form solution. In fact, for the G-M robust loss, the solution of (3.21) is just (3.20). For the truncated quadratic loss, problem (3.21) reads

$$
\min _ {w _ {i} \in [ 0, 1 ]} (1 - w _ {i}) \beta_ {i} ^ {2} + w _ {i} r _ {i} ^ {2} (\boldsymbol {x})
$$

and admits a closed-form solution

$$
w _ {i} ^ {\star} = \left\{ \begin{array}{l l} 1 & \text {if r_{i} ^{2} (\boldsymbol{x}) <   \beta_{i} ^{2}} \\ 0 & \text {if r_{i} ^{2} (\boldsymbol{x) > \beta_{i} ^{2}} .} \\ [ 0, 1 ] & \text {otherwise} \end{array} \right.
$$

In summary, the t-th iteration of IRLS in the context of B-R duality alternates between two steps

1 Variable update: solve a weighted least squares problem using the current weights $w _ { i } ^ { ( t ) }$

$$
\boldsymbol {x} ^ {(t)} \in \underset {\boldsymbol {x}} {\arg \min} \sum_ {i} w _ {i} ^ {(t)} r _ {i} ^ {2} (\boldsymbol {x}).\tag{3.22}
$$

2 Weight update: update the weights using $\mathbf { \boldsymbol { x } } ^ { ( t ) }$

$$
w _ {i} ^ {(t + 1)} \in \underset {w _ {i} \in [ 0, 1 ]} {\arg \min} \Phi_ {\rho} (w _ {i}) + w _ {i} r _ {i} ^ {2} (\boldsymbol {x} ^ {(t)}), i = 1, \ldots , N.\tag{3.23}
$$

The weight update rule obtained via B-R duality actually matches the popular weight update rule (3.12). Interestingly, instantiating the above IRLS algorithm in SLAM using the G-M robust loss leads to the dynamic covariance scaling algorithm [11], which has been proposed in the context of outlier-robust SLAM.

## 3.3.4 Graduated Non-Convexity

The previous section leveraged Black-Rangarajan duality and alternating minimization to derive the IRLS framework that alternates in solving (3.22) and (3.23). However, due to the non-convexity of common robust losses, the convergence of the IRLS framework can be highly sensitive to the quality of initialization, $i . e .$ , how close is $\mathbf { x } ^ { ( 0 ) }$ to the optimal solution of (3.9) or how well does $w _ { i } ^ { ( 0 ) }$ reflect the inlieroutlier membership of each measurement. For example, [1006] showed that IRLS with the truncated quadratic loss and the Geman-McClure loss might fail when there are as little as 10% outliers in the measurements $( c f .$ also with our results in Figure 3.7).

In this section, we introduce the Graduated Non-Convexity (GNC) algorithms, that can make IRLS significantly less sensitive to the quality of initialization. Given a robust cost function $\rho ,$ , the basic idea of GNC is to create a smooth version of $\rho ,$ denoted as $\rho _ { \mu }$ , using a scalar smoothing factor $\mu .$ Tuning $\mu$ controls the amount of non-convexity in $\rho _ { \mu } \colon \rho _ { \mu }$ is convex at one end of the spectrum and recovers the original $\rho$ at the other end of the spectrum. Let us illustrate this using two examples.

Example 3.6 (GNC Truncated Quadratic Loss) Consider the GNC truncated

quadratic loss function

$$
\rho_ {\mu} (r _ {i} (\pmb {x})) = \left\{ \begin{array}{l l} r _ {i} ^ {2} (\pmb {x}) & \mathrm{if} r _ {i} ^ {2} (\pmb {x}) \in \left[ 0, \frac {\mu}{\mu + 1} \beta_ {i} ^ {2} \right] \\ 2 \beta_ {i} | r _ {i} (\pmb {x}) | \sqrt {\mu (\mu + 1)} - \mu (\beta_ {i} ^ {2} + r _ {i} ^ {2} (\pmb {x})) & \mathrm{if} r _ {i} ^ {2} (\pmb {x}) \in \left[ \frac {\mu}{\mu + 1} \beta_ {i} ^ {2}, \frac {\mu + 1}{\mu} \beta_ {i} ^ {2} \right] \\ \beta_ {i} ^ {2} & r _ {i} ^ {2} (\pmb {x}) \in \left[ \frac {\mu + 1}{\mu} \beta_ {i} ^ {2}, + \infty \right]. \end{array} \right.\tag{3.24}
$$

$\rho _ { \mu }$ is convex for $\mu$ approaching zero and retrieves the truncated quadratic loss in (3.13) for $\mu$ approaching infinity.

Example 3.7 (GNC Geman-McClure Loss) Consider the GNC Geman-McClure loss function

$$
\rho_ {\mu} (r _ {i} (\pmb {x})) = \frac {\mu \beta_ {i} ^ {2} r _ {i} ^ {2} (\pmb {x})}{\mu \beta_ {i} ^ {2} + r _ {i} ^ {2} (\pmb {x})}.\tag{3.25}
$$

$\rho _ { \mu }$ is convex for $\mu$ approaching and recovers the G-M loss (3.17) when $\mu = 1$

![](images/d5c45e32a41fb6e784e6325650985c2586a323d0397f6ad629c99100d34d40ba.jpg)  
(a)

![](images/601276569c745a3004affebd09355ebb79087d71f04fbcfc5891d0779d5a58c8.jpg)  
(b)  
Figure 3.8 Graduated Non-Convexity with control parameter µ for (a) Truncated Quadratic loss and (b) Geman-McClure loss. [1222] (©2020 IEEE)

Figure 3.8(a) and (b) plot the GNC truncated quadratic loss and the GNC Geman-McClure loss, respectively. Observe how increasing or decreasing the control parameter $\mu$ adds more non-convexity to the function.

One nice property of the smoothed GNC functions in (3.24) and (3.25) is that the B-R duality still applies. For the GNC truncated quadratic loss (3.24), applying B-R duality leads to the outlier process

$$
\Phi_ {\rho_ {\mu}} (w _ {i}) = \frac {\mu (1 - w _ {i})}{\mu + w _ {i}} \beta_ {i} ^ {2}.
$$

For the GNC Geman-McClure loss (3.25), applying B-R duality leads to the outlier

process

$$
\Phi_ {\rho_ {\mu}} (w _ {i}) = \mu \beta_ {i} ^ {2} (\sqrt {w _ {i}} - 1) ^ {2}.
$$

We are now ready to state the GNC algorithm, which at each iteration performs three steps

1 Variable update: solve a weighted least squares problem using the current weights $w _ { i } ^ { ( t ) }$

$$
\boldsymbol {x} ^ {(t)} \in \arg \min _ {\boldsymbol {x}} \sum_ {i} w _ {i} ^ {(t)} r _ {i} ^ {2} (\boldsymbol {x}).\tag{3.26}
$$

2 Weight update: update the weights using $\mathbf { \boldsymbol { x } } ^ { ( t ) }$

$$
w _ {i} ^ {(t + 1)} \in \underset {w _ {i} \in [ 0, 1 ]} {\arg \min} \Phi_ {\rho_ {\mu}} (w _ {i}) + w _ {i} r _ {i} ^ {2} (\boldsymbol {x} ^ {(t)}), i = 1, \ldots , N.\tag{3.27}
$$

3 Control parameter update: Increase or decrease $\mu$ to add more nonconvexity to $\rho _ { \mu }$

The GNC algorithm is similar to the IRLS algorithm, except that it starts with a convex, smoothed function $\rho _ { \mu }$ and then iteratively updates the control parameter $\mu$ to gradually add more non-convexity to $\rho _ { \mu }$ to approach the original loss function $\rho .$ Depending on the definition of the smoothed loss $\rho _ { \mu }$ , one would recover the original $\rho$ by either increasing or decreasing $\mu .$ . For instance, the smoother GNC truncated quadratic loss recovers the original truncated quadratic loss when $\mu$ is large, hence $\mu$ is increased by a constant factor $\gamma > 1$ at each GNC iteration $( e . g . , \gamma = 1 . 4 $ in [1222]). Conversely, the smoother Geman-McClure loss recovers the original GM loss when $\mu$ is close to 1, hence $\mu$ is divided by $\gamma > 1$ at each GNC iteration.

Figure 3.9 showcases the SLAM trajectories obtained by applying GNC on the same three datasets of Figure 3.6, with two diferent robust losses: the Geman-McClure loss and the truncated quadratic loss. We implemented GNC using GT-SAM’s GNCOptimizer. By comparing the figure with Figure 3.6 and Figure 3.7 we observe that GNC ensures better convergence $( i . e . ,$ , it is less prone to being stuck in local minima) and recovers fairly accurate trajectories in all the three datasets. While GNC has been shown to be extremely resilient to outliers (e.g., it has shown to tolerate around 80-90% incorrect loop closures in real-world problems [1222, 165]), we remark that the approach does not provide any convergence guarantees. Moreover, its performance has been empirically seen to be problemdependent, and while it leads to superb performance in pose-graph optimization problems, its performance largely degrades in other perception problems [1005].

Below we showcase a problem when GNC fails and also show that combining front-end and back-end outlier rejection can be beneficial. Towards this goal, we are going to consider a slightly more challenging SLAM setup compared to the ones discussed above. Earlier in this chapter, we considered pose-graph optimization problems $( e . g .$ , Figure 3.1) where the odometry is reliable but there might be outliers in the loop closures or in the landmark measurements. This setting essentially assumes the presence of an “odometry backbone” that largely simplifies the problem by providing a set of trusted measurements while also allowing building an initial guess for the robot poses. While this assumption is realistic in many problems,<sup>14</sup> certain SLAM applications might lack an odometry backbone. For instance, certain odometry sensors might produce incorrect measurements, $e . g .$ , due to wheel slippage in wheel odometry, or incorrect lidar alignment in lidar odometry. Another example arises in multi-robot SLAM, where each robot has an odometry backbone, but the overall SLAM problem (including the trajectory of all robots) does not [728].

M3500  
![](images/b05ceda71ff0e3dee01d0b5fc5336292fd937615309e129bd5b19a6083a6bc62.jpg)  
(a)  
SubT

![](images/2509e7220eb57b3d6e615fb69bd6ab471a68341f555208c627fe8c275c27d372.jpg)  
Victoria Park

![](images/cae6574d230e4a3c126702533cde19830151b1098536973603ff71a249b25185.jpg)  
(b)  
(c)

![](images/5f3d80b8c9993f563c9cce32d7c19178eeaba68ef463a812f5e44b2b4eaee428.jpg)  
(d)

![](images/a47812bf602bcbfc1e8bea276ffc29ff1c887a40d4a7f1088be0855292b44bf1.jpg)

![](images/def2c00b9208c3e958416444ea23922ce5d260c838da2437dadb5e5f110d6877.jpg)

![](images/38203cc950fa0189b22b31bf9677bf1ed60c9fb522cd4ccd4b3fc68fb3088bc4.jpg)  
(g)

(f)  
(e)  
![](images/78e6ddafe7c9a558f683a314e4fc185ceb9b843eaef1d16d65526526611b7f65.jpg)  
(h)

![](images/060b8e619f9c08bb2f70d19018afb69e81988d5c7556b91ecd2021e099208f4b.jpg)  
(i)  
Figure 3.9 Solving SLAM problems with outliers using robust loss functions and graduated non-convexity (GNC): (a)-(c) Ground truth trajectories for the M3500, SubT, and Victoria Park datasets. (d)-(f) Trajectory estimates obtained using the Geman-McClure loss. (g)- (i) Trajectory estimates obtained using the truncated quadratic loss. Measurements are visualized as colored edges. In particular, an edge is colored in gray if it is correctly classified as inlier or outlier by the optimization $( i . e . ,$ an inlier that falls in the quadratic region of the Huber or truncated quadratic loss); it is colored in red if it is an outlier incorrectly classified as an inlier (a “false positive”); it is colored in blue if it is an inlier incorrectly classified as an outlier (a “false negative”).

M3500  
![](images/d2882ed1424ef18300c55e7d00e57729eb0b84a2385d828ac2329d94ed020750.jpg)  
(a)

SubT  
![](images/5df8cba2e5315e1f2caec4c78b5e47f41ddf1b2c5205778b831bd48f8663c940.jpg)  
Victoria Park

![](images/e09061bf60bd4e86e3bbc576d7dd23edebb5227a069c30cd3eec5248284cba8d.jpg)

![](images/fa5804728b14230829381ece9f392d23b9b0ec5990742ae2c6676d1649039eb1.jpg)

(b)  
![](images/a7c09461b4a23d467b3bcb78ca6b10fa61d1ea1a63ea1b27814eee4b680fd151.jpg)

(c)  
![](images/4445cccec0d0f23f181f9b78da1f6e410799d47cb933104b566421957008e984.jpg)

(d)  
![](images/90e841f1a16ac212ff2be7066b957819a997ccc70de5f3501f3fd36f8ab00ec2.jpg)  
(g)

(e)  
![](images/3c2bf85279f9d3bf422848becd821a386a8abe8ac06de369855d606430bde9d6.jpg)  
(h)

(f)  
![](images/aaca4bf1d9cb5663a25d86c161fac9f126bdffe2719cb8e688575f342d59937f.jpg)  
(i)  
Figure 3.10 SLAM problems with outliers in both the loop closures and landmark measurements, as well as the odometry: (a)-(c) Trajectory estimates obtained using GNC with the truncated quadratic loss. (d)-(f) Trajectory estimates obtained using PCM for front-end outlier rejection followed by least squares optimization. (g)-(i) Trajectory estimates obtained using PCM for front-end outlier rejection followed by GNC with truncated quadratic loss.

Robustly solving SLAM problems where both odometry and loop closure measurements can be outliers is extremely hard. To showcase the shortcomings of the approaches we discussed, in this more complex setting with potentially incorrect odometry measurements, we first modify the pose-graph problems in Figure 3.1 by corrupting two randomly selected odometry measurements. Then we attempt to solve the problem with GNC. In particular, in GNC we fix all odometry measurements except the two potentially corrupted measurements as inliers. Figure 3.10 shows the trajectories obtained by solving the problem with GNC. GNC fails due to the corrupted initialization and converges to a local minimum by categorizing all measurements (including inliers) as outliers (blue edges). The same figure also shows the result of using PCM to filter out gross outliers before using a least-squares back-end. PCM is agnostic to the initial guess, hence is able to converge to better solutions in the case of the SubT and Victoria Park datasets (Figure 3.10 (e) and (f)). Interestingly, we get best results in the SubT and Victoria Park datasets by combining front-end outlier rejection (PCM) with GNC. At the same time, all three methods (GNC, PCM, and PCM+GNC) fail to converge to acceptable solutions in the M3500 dataset, confirming the hardness of this SLAM setup and the limitations of existing SLAM algorithms in terms of outlier rejection.

## 3.4 Further Readings & Recent Trends

Consensus Maximization. While in this chapter we discussed the most basic instantiation of a RANSAC algorithm (according to the initial proposal in [328]), it is worth mentioning that the literature ofers many RANSAC variants, including variants that refine estimates through local optimization [216], use better scores (rather than the size of the consensus set) in the RANSAC iterations (e.g., MLESAC [872]), or bias the sampling in the RANSAC iterations (e.g., PROSAC [215]). The recent literature also includes diferentiable variants of RANSAC [1174] and variants that attempt to find the inliers when the parameter γ in (3.4) is unknown (e.g., [46]). A recent survey and evaluation of RANSAC variants can be found in [1109].

Beyond RANSAC, the literature also includes approaches for exact consensus maximization, typically based on branch-and-bound [64, 436, 1290, 651, 1026, 203, 507, 132, 1226, 1224]. Despite its global optimality guarantees, branch-and-bound has exponential runtime in the worst case and does not scale to high-dimensional problems.

Pairwise Consistency Maximization. The PCM approach described in Section 3.2.2 was originally proposed by Mangelson et al. [728] in the context of multirobot SLAM, where this approach showed particular promise. Graph-theoretic outlier rejection has also been investigated in computer vision. San Segundo and Artieda [967] build an association graph and find the maximum clique for 2D image feature matching. Perera and Barnes [869] segment objects under rigid-body motion with a clique formulation. Leordeanu and Hebert [639] establish image matches by finding strongly-connected clusters in the correspondence graph with an approximate spectral method. Enqvist et al. [309] develop an outlier rejection algorithm for 3D-3D and 2D-3D registration based on approximate vertex cover. Yang and Carlone [1217], Yang et al. [1223] and Bustos et al. [133] investigate graphtheoretic outlier rejection based on maximum clique for 3D-3D registration. The idea of checking consistency across a subset of measurements also arises in Latif et al. [628], which perform loop closure outlier rejection by clustering measurements together and checking for consistency using a Chi-squared-based test. The PCM paper [728], similarly to the discussion in this chapter, focuses on pairwise consistency. More recently, PCM has been extended to group-k consistency (i.e., the case where the consistency constraint (3.7) involves k measurements instead of only 2 measurements) in [1005, 1006, 332]. These papers essentially generalize the notion of consistency graphs to consistency hypergraphs, where each hyper-edge involves k nodes. Related work also considers soft variations of the maximum clique problem, where the binary condition (3.7) is relaxed to produce continuous weights on the edges of the consistency graph [712, 711]. These methods have been used in practical applications, including subterranean exploration [298], lidar point-cloud localization [713], multi-robot metrics-semantic mapping [1095], and global localization in unstructured environments [33].

Alternating Minimization and Graduated Non-Convexity. M-estimation has been a popular approach for robust estimation in robotics [99] and vision [978, 173]. Tavish and Barfoot [1075] investigate the performance of diferent loss functions. Several papers investigate formulations with auxiliary variables as the one in (3.16), without realizing the connection to M-estimation provided by the Black-Rangarajan duality (Theorem 3.4). For instance, S¨underhauf and Protzel [1059], S¨underhauf and Protzel [1058] and Agarwal et al. [11] augment the problem with latent binary variables responsible for deactivating outliers. Lee et al. [637] use expectation maximization. Olson and Agarwal [827] use a max-mixture distribution to approximate multi-modal measurement noise. Recently, Barron [57] proposes a single parametrized function that generalizes a family of robust loss functions in Mestimation. Chebrolu et al. [174] design an expectation-maximization algorithm to simultaneously estimate the unknown quantity x and choose a suitable robust loss function ρ. The graduated non-convexity algorithm was first introduced in [86, 85] for outlier rejection in early vision applications; more recently, the algorithm was used for point cloud registration [1297, 1223], SLAM [1222], and other applications [34]. Recently, Peng et al. [860] have proposed an algorithm similar to GNC and IRLS, that is based on the idea of smooth majorization in optimization and can be applied to a broad set of robust losses. Moreover, Peng et al. [860] derives global and local convergence guarantees for GNC.

Certifiable Algorithms. The algorithms described so far can be broadly divided into two categories: (i) fast heuristics (e.g., RANSAC or local solvers for M-estimation), which are eficient but provide little performance guarantees, and (ii) global solvers (e.g., branch-and-bound), which ofer optimality guarantees but scale poorly with the problem size. Recent years have seen the advent of a new type of methods, called certifiable algorithms, that try to strike a balance between tractability and optimality. Certifiable algorithms relax non-convex robust estimation problems into convex semidefinite programs (SDP),<sup>15</sup> whose solutions can be obtained in polynomial time and provide readily checkable a posteriori global optimality certificates. Certifiable algorithms for robust estimation have been proposed in the context of rotation estimation [1218], 3D-3D registration [1223], and posegraph optimization [618, 153]. A fairly general approach to derive certifiable algorithms for problems with outliers is described in [1219, 1221], while connections with parallel work in statistics is discussed in [147]. With few notable exceptions, these algorithms, albeit running in polynomial time, are still computationally expensive and typically much slower than heuristics methods. In some cases, the insights behind these algorithms can be used to certify optimality of a solution obtained with a fast heuristic [1219], hence getting the best of both worlds.

Chen Wang, Krishna Murthy Jatavallabhula, and Mustafa Mukadam

As presented in Chapter I, the design of a contemporary SLAM system generally adheres to a front-end and back-end architecture. In this structure, the front-end is typically responsible for pre-processing sensor data and generating an initial estimate of the robot’s trajectory and the map of the environment, while the backend refines these initial estimates to improve overall accuracy. Recent advances in machine learning have provided new approaches, based on deep neural networks, that have the potential to enhance some of the functionalities in the SLAM frontend. For instance, deep learning-based methods can exhibit impressive performance in feature detection and matching [971, 267, 1267] and front-end motion estimation [1167, 1082]. These methods train a neural network from a large dataset of examples, and then make estimations without being explicitly programmed to perform the task. Meanwhile, geometry-based techniques persist as an essential element for the SLAM back-end, primarily due to their generality and efectiveness in producing a globally consistent estimate by solving an optimization problem [1208].

While in principle one could just “plug” a learning-based SLAM front-end in the SLAM architecture and feed the corresponding outputs to the back-end, the use of learning-based techniques opens the door for a less unidirectional information exchange. In particular, the back-end can now provide feedback to the front-end, enabling it to learn directly from the back-end estimates in a way that the two modules can more harmoniously cooperate to reduce the estimation errors. Reconciling geometric approaches with deep learning to leverage their complementary strengths is a common thread in a large body of recent work in SLAM. In particular, an emerging trend is to diferentiate through geometry-based optimization problems arising in the SLAM back-end. Intuitively, diferentiating through an optimization problem allows understanding how the optimal solution of that problem (e.g., our SLAM estimate) depends on the parameters of that problem —in our case, the measurements produced by a learning-based front-end; this in turns allows optimizing the front-end to maximize the SLAM accuracy. One could think about this as a bilevel optimization problem, i.e., an upper-level optimization process subject to a lower-level optimization —in particular, a neural network based-optimization to train the front-end, subject to a geometry-based optimization that computes the SLAM solution for a given front-end output.

The ability to compute gradients end-to-end through an optimization is the core of solving a bilevel optimization problem, which allows neural models to take advantage of geometric priors captured by the optimization. The flexibility of such a scheme has led to promising state-of-the-art results in a wide range of applications such as structure from motion [1080], motion planning [81, 1216], SLAM [511, 1082], BA [1068, 1267], state estimation [1244, 182], and image alignment [716].

In this chapter, we illustrate the basics of how to diferentiate through nonlinear least squares problems, such as the ones arising in SLAM. Specifically, Section 4.1 restates the non-linear least square (NLS) problem. Section 4.2 describes how to diferentiate through the NLS problem. Section 4.3 shows how to diferentiate problems defined on manifold. Section 4.4 discusses numerical challenges of the above diferentiation and introduces related machine learning libraries. Finally, Section 4.5 provides examples of diferentiable optimization in contemporary SLAM systems.

## 4.1 Recap on Nonlinear Least Squares

Non-linear least squares (NLS) estimate the parameters of a model by minimizing the sum of the squares of the mismatch between observed values and those predicted by the model. Unlike linear least squares, NLS involve a model that is non-linear in the parameters. Beyond our factors graphs in Chapter 1, this approach is widely used in many fields such as statistics, physics, and engineering, where it is useful for fitting complex models to data when the relationship between variables is not straightforward, enabling more accurate and robust predictions.

Specifically, NLS aim to find variables $\pmb { x } \in \mathbb { R } ^ { n }$ by solving:

$$
\boldsymbol {x} ^ {*} = \underset {\boldsymbol {x}} {\arg \min} \mathcal {L} (\boldsymbol {x}) = \underset {\boldsymbol {x}} {\arg \min} \frac {1}{2} \sum_ {i} | | \underbrace {w _ {i} \boldsymbol {c} _ {i} (\boldsymbol {x} _ {i})} _ {\boldsymbol {r} _ {i} (\boldsymbol {x} _ {i})} | | ^ {2},\tag{4.1}
$$

where the objective $\scriptstyle { \mathcal { L } } ( x )$ is a sum of squared vector-valued residual terms $\mathbf { \nabla } _ { \mathbf { r } _ { i } }$ , each a function of $\mathbf { { \pmb x } } _ { i } \subset { \pmb x }$ that are (non-disjoint) subsets of the optimization variables $\textbf { \em x } = \{ \pmb { x } _ { i } \}$ . While for now we assume $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { i } }$ to be vectors, later in the chapter we generalize the discussion to the case where the variables belong to a manifold. For flexibility, here we represent a residual ${ \pmb r } _ { i } ( { \pmb x } _ { i } ) = w _ { i } { \pmb c } _ { i } ( { \pmb x } _ { i } )$ as a product of a weight $w _ { i }$ and vector cost $c _ { i }$

As explained in Chapter 1, a NLS is normally solved by iteratively linearizing the nonlinear objective around the current variables to get the linear system $\begin{array} { r } { ( \sum _ { i } \pmb { J } _ { i } ^ { \top } \pmb { J } _ { i } ) \delta \pmb { x } = ( \sum _ { i } \pmb { J } _ { i } ^ { \top } \pmb { r } _ { i } ) } \end{array}$ , then solving the linear system to find the update δx, and finally updating the variables ${ \pmb x }  { \pmb x } + \delta { \pmb x }$ , until convergence. We have also commented in Chapter 2 that the addition in the update step is more generally a retraction mapping for variables that belong to a manifold. In the linear system,

![](images/9b6b12b1a1fc2f68504689c35738c5b599e7879244940e04019aecab1553f5c4.jpg)  
Figure 4.1 A modern SLAM system often involves both neural networks and nonlinear least squares. To eliminate compound errors introduced by optimizing the two modules separately, we can optimize the system in an end-to-end manner by formulating the entire system as a bilevel optimization, which involves an upper-level cost and a lower-level cost.

$J _ { i } = [ \partial { \pmb r } _ { i } / \partial { \pmb x } ]$ are the Jacobians of residuals with respect to the variables. This iterative method above, called Gauss-Newton (GN), is a nonlinear optimizer that is (approximately) second-order, since $\sum _ { i } J _ { i } ^ { \top } J _ { i }$ is an approximation of the Hessian. To improve robustness and convergence, variations like Levenberg-Marquardt (LM) dampen the linear system, while others adjust the step size for the update with line search, $e . g .$ , Dogleg introduced in Chapter 1.

## 4.2 Diferentiation Through Nonlinear Least Squares

To seamlessly merge deep learning with nonlinear least squares, diferentiable nonlinear least squares (DNLS) are often required to solve the optimization problem illustrated in Figure 4.1. This necessitates gradients of the solution $\pmb { x } ^ { * }$ with respect to any upper-level neural model parameters y that parameterize the objective $\mathcal { U } ( \pmb { x } ; \pmb { y } )$ and, in turn, any costs $c _ { i } ( { \pmb x } _ { i } ; { \pmb y } )$ or initialization for variables ${ \pmb x } _ { \mathrm { i n i t } } ( { \pmb y } )$ . The goal is to learn these parameters y end-to-end with a lower-level learning objective $\mathcal { L }$ defined as a function of x. This results in a Bilevel Optimization (BLO), which can be written as:

$$
\boldsymbol {y} ^ {*} = \arg \min _ {\boldsymbol {y} \in \Theta} \mathcal {U} (\boldsymbol {y}, \boldsymbol {x} ^ {*}),\tag{4.2a}
$$

$$
\text { s.   t. } \quad \boldsymbol {x} ^ {*} = \arg \min _ {\boldsymbol {x} \in \Psi} \mathcal {L} (\boldsymbol {y}, \boldsymbol {x}),\tag{4.2b}
$$

where $\mathcal { L } : \mathbb { R } ^ { m } \times \mathbb { R } ^ { n } $ R is a lower-level (LL) cost, $\mathcal { U } : \mathbb { R } ^ { m } \times \mathbb { R } ^ { n } $ R is a upper-level (UL) cost, $\pmb { x } \in \Psi$ and $\pmb { y } \in \Theta$ are the feasible sets.

In practice, the variables x are often parameters with explicit physical meanings such as camera poses, while y are parameters without physical meanings such as weights in a neural network. We next present two examples to explain this.

Example 4.1 (Visual SLAM with Learned Features) Imagine a SLAM system that leverages a neural network (parameterized by y) for feature extraction/matching, while utilizing BA for pose estimation (parameterized by x), which take the feature matching as an input. In this example, the UL cost (4.2a) can be feature matching error for optimizing the network, while the LL cost L (4.2b) can be the reprojection error for BA. Intuitively, the optimal solution $\mathbf { \nabla } _ { \mathbf { \mathcal { X } } } \ast \mathbf { \ v { x } }$ for the camera poses and landmark positions plays the role of a supervisory signal in the neural network training. Therefore, optimizing the BLO (4.2) allows us to further reduce the matching error via back-propagating the BA reprojection errors [1267].

Example 4.2 (PGO) Imagine a SLAM system that uses a neural network for front-end pose estimation, while leverages PGO as the back-end to eliminate odometry drifts. In this example, both UL and LL costs can be the pose-graph error. The diference is the UL cost optimizes the network parameterized by y, while the LL cost optimizes the camera poses parameterized by x. As a result, the frontend network can leverage global geometric knowledge obtained through pose-graph optimization by back-propagating the pose residuals from the back-end PGO [346].

BLO is a long-standing and well-researched problem [1148, 517, 679]. Solving a BLO often relies on gradient-descent techniques. Specifically, the UL optimization performs updates in the form $\pmb { y }  \pmb { y } + \delta \pmb { y }$ , where δy is a step in the direction of the negative gradient. Therefore, we need to compute the gradient of with respect to the UL variable y, which can be written as

$$
\nabla_ {\boldsymbol {y}} \mathcal {U} = \frac {\partial \mathcal {U} (\boldsymbol {y} , \boldsymbol {x} ^ {*})}{\partial \boldsymbol {y}} + \frac {\partial \mathcal {U} (\boldsymbol {y} , \boldsymbol {x} ^ {*})}{\partial \boldsymbol {x} ^ {*}} \frac {\partial \boldsymbol {x} ^ {*} (\boldsymbol {y})}{\partial \boldsymbol {y}},\tag{4.3}
$$

where the term $\frac { \partial \pmb { x } ^ { * } ( \pmb { y } ) } { \partial \pmb { y } }$ involves indirect gradient computation. Since other direct gradient terms in (4.3) are easy to obtain, the challenge of solving a BLO (4.2) is to compute the term $\frac { \partial \pmb { x } ^ { * } ( \pmb { y } ) } { \partial \pmb { y } }$ . For this purpose, a series of techniques have been developed from either explicit or implicit perspectives. This involves recurrent differentiation through dynamical systems or implicit diferentiation theory, which are often referred to as unrolled diferentiation and implicit diferentiation, respectively. These algorithms have been summarized in [679, 1148] and here we list a generic framework incorporating both methods in Algorithm 1. We next introduce unrolled diferentiation and implicit diferentiation.

## 4.2.1 Unrolled Diferentiation

Unrolled Diferentiation needs Automatic Diferentiation (AutoDif) through the LL optimization to solve a BLO problem. Specifically, given an initialization ${ \bf { x } } _ { 0 } =$ $\Phi _ { 0 } ( \pmb { y } )$ at step t = 0, the iterative process of unrolled LL optimization is

$$
\pmb {x} _ {t} = \Phi_ {t} (\pmb {x} _ {t - 1}; \pmb {y}), \quad t = 1, \dots , T,\tag{4.4}
$$

<div class="mineru-algorithm" style="white-space: pre-wrap; font-family:monospace;">
Algorithm 1 Solving BLO by Unrolled Differentiation or Implicit Differentiation.

1: Initialization:  $y_{0}$ ,  $x_{0}$ .
2: while Not Convergent ( $\|y_{k+1}-y_{k}\|$  is large enough) do
3: Obtain  $x_{T}$  by solving (4.2b) by a generic optimizer O with T steps.
4: Efficient estimation of upper-level gradients in (4.3) via
5: Unrolled Differentiation:  $\hat{\nabla}_{y_{k}}U=\frac{\partial U(y_{k},x_{T})}{\partial y_{k}}$  via AutoDiff in (4.7).
6: Implicit Differentiation (Algorithm 2): Compute
 $\hat{\nabla}_{y_{k}}\mathcal{U}=\frac{\partial\mathcal{U}}{\partial y_{k}}\bigg|_{x_{T}}+\frac{\partial\mathcal{U}}{\partial x^{*}}\frac{\partial x^{*}}{\partial y_{k}}\bigg|_{x_{T}}$ ,
where the implicit derivatives  $\frac{\partial x^{*}}{\partial y_{k}}$  can be obtained by solving an equation derived via lower-level optimality conditions (surveyed in following sections).
7:
8: Compute  $y_{k+1}$  via gradients using  $\hat{\nabla}_{y_{k}}U$ .
9: end while
</div>

where $\Phi _ { t }$ denotes an updating scheme based on the LL problem at the t-th step and $T$ is the number of iterations. One updating scheme is the gradient descent:

$$
\Phi_ {t} (\boldsymbol {x} _ {t - 1}; \boldsymbol {y}) = \boldsymbol {x} _ {t - 1} - \eta_ {t} \cdot \frac {\partial \mathcal {L} (\boldsymbol {x} _ {t - 1} , \boldsymbol {y})}{\partial \boldsymbol {x} _ {t - 1}},\tag{4.5}
$$

where $\eta _ { t }$ is a learning rate and the term $\frac { \partial \mathcal { L } ( \pmb { x } _ { t - 1 } , \pmb { y } ) } { \partial \pmb { x } _ { t - 1 } }$ can be computed from AutoDif.<sup>1</sup> Therefore, we can compute the $\nabla _ { \boldsymbol { y } } \boldsymbol { \mathcal { U } } ( \boldsymbol { y } )$ by substituting $\mathbfit { \mathbf { x } } _ { T }$ approximately for $\mathbf { \boldsymbol { x } } ^ { * }$ and the full unrolled system can be defined as

$$
\boldsymbol {x} ^ {*} \approx \boldsymbol {x} _ {T} = \Phi (\boldsymbol {y}) = \left(\Phi_ {T} \circ \dots \circ \Phi_ {1} \circ \Phi_ {0}\right) (\boldsymbol {y}),\tag{4.6}
$$

where the symbol denotes the function composition. As a result, we only need to consider the following problem instead of a bilevel optimization in (4.2):

$$
\min _ {\boldsymbol {y} \in \Theta} \mathcal {U} (\boldsymbol {y}, \Phi (\boldsymbol {y})),\tag{4.7}
$$

which needs to compute $\frac { \partial \Phi ( \pmb { y } ) } { \partial \pmb { y } }$ via AutoDif instead of calculating (4.3). It is worth noting that there exist two approaches for computing the recurrent gradients, one of which corresponds to backward propagation in a reverse-mode way [857], and the other corresponds to the forward-mode way [928]. We omit the details of the two approaches of AutoDif and refer the readers to the AutoDif libraries such as $\mathrm { P y }$ Torch [850] for deep learning and $\mathrm { P y }$ Pose [1145] and Theseus [873] for SLAM. A review of these approaches can also be found in Liu et al. [679].

## 4.2.2 Truncated Unrolled Diferentiation

The reverse and forward modes are two precise recurrent gradient calculation methods but are time-consuming with the full iterative propagation. This is due to the complicated long-term dependencies of the UL problem on $\mathbf { \nabla } _ { \mathbf { x } _ { t } , }$ where $t = 0 , 1 , \cdots , T$ This dificulty is further aggravated when both y and x are high-dimensional vectors. To overcome this challenge, the truncated unrolled diferentiation has been investigated as a way to compute high-quality approximate gradients with significantly less computation time and memory. Specifically, by ignoring the long-term dependencies and approximating the gradient of (4.5) with partial history, $i . e . ,$ storing only the last M iterations $( t = T , T - 1 , \cdot \cdot \cdot , T - M )$ , we can significantly reduce the time and space complexity. It has been proved by Shaban et al. [992] that using fewer backward steps to compute the gradients could perform comparably to optimization with the exact one, while requiring much less memory and computation.

In case of more stringent computational and memory constraints, truncated unrolled diferentiation is still often a bottleneck in modern robotic applications. Therefore, researchers have also tried to further simplify the truncated diferentiation by only performing a one-step iteration in (4.4) to remove the recursive structure [676], i.e.,

$$
\nabla_ {\boldsymbol {y}} \mathcal {U} = \frac {\partial \mathcal {U} (\boldsymbol {y} , \boldsymbol {x} _ {1} (\boldsymbol {y}))}{\partial \boldsymbol {y}} + \frac {\partial \mathcal {U} (\boldsymbol {y} , \boldsymbol {x} _ {1} (\boldsymbol {y}))}{\partial \boldsymbol {x} _ {1}} \frac {\partial \boldsymbol {x} _ {1} (\boldsymbol {y})}{\partial \boldsymbol {y}},\tag{4.8}
$$

where the term $\frac { \partial { \pmb x } _ { 1 } ( { \pmb y } ) } { \partial { \pmb y } }$ is a Hessian that can be calculated from (4.5) as

$$
\frac {\partial \boldsymbol {x} _ {1} (\boldsymbol {y})}{\partial \boldsymbol {y}} = - \frac {\partial^ {2} \mathcal {L} (\boldsymbol {x} _ {0} , \boldsymbol {y})}{\partial \boldsymbol {x} _ {0} \partial \boldsymbol {y}}.\tag{4.9}
$$

Since calculating a Hessian is time-consuming in some applications, we can resort to numerical solutions that apply small perturbations to the variables x and calculate an approximation of the second term in (4.8) as a whole:

$$
\frac {\partial \mathcal {U} (\boldsymbol {y} , \boldsymbol {x} _ {1} (\boldsymbol {y}))}{\partial \boldsymbol {x} _ {1}} \frac {\partial \boldsymbol {x} _ {1} (\boldsymbol {y})}{\partial \boldsymbol {y}} \approx \frac {\frac {\partial \mathcal {L} (\boldsymbol {x} _ {0} ^ {+} , \boldsymbol {y})}{\partial \boldsymbol {y}} - \frac {\mathcal {L} (\boldsymbol {x} _ {0} ^ {-} , \boldsymbol {y})}{\partial \boldsymbol {y}}}{2 \epsilon},\tag{4.10}
$$

where $\epsilon$ is a small scalar and $\begin{array} { r } { \pmb { x } _ { 0 } ^ { \pm } = \pmb { x } _ { 0 } \pm \epsilon \frac { \partial \mathcal { U } ( \pmb { y } , \pmb { x } _ { 1 } ( \pmb { y } ) ) } { \partial \pmb { x } _ { 1 } } } \end{array}$ is a small perturbation. This bypasses an explicit calculation of the Jacobian $\frac { \partial { \pmb x } _ { 1 } ( { \pmb y } ) } { \partial { \pmb y } }$ . Nevertheless, we need to pay attention to the perturbation model if non-Euclidean variables are involved, $e . g .$ variables belonging to Lie Groups. Fortunately, the AutoDif of Lie Group for Hessian-vector and Jacobian-vector multiplications are supported in modern libraries, such as PyPose [1145], which will be introduced in Section 4.4.

## 4.2.3 Implicit Diferentiation

It is intuitive that the term $\frac { \partial \pmb { x } ^ { * } ( \pmb { y } ) } { \partial \pmb { y } }$ in (4.3) is dependent on the LL cost (4.2b), thus implicit diferentiation can be used to derive a solution to the gradient.

In calculus, implicit diferentiation refers to the method that makes use of the chain rule to diferentiate an implicit function. To diferentiate an implicit function $y ( x )$ , defined by an equation $R ( x , y ) = 0$ , it is not generally possible to solve it explicitly for y and then diferentiate. Instead, one can totally diferentiate $R ( x , y ) =$ 0 with respect to x and then solve the resulting linear equation for $\frac { \mathrm { d } y } { \mathrm { d } x }$ to explicitly get the derivative in terms of x and y. For instance, consider an implicit function $x + y + 5 = 0$ , diferentiating it with respect to x on its both sides gives $\begin{array} { r } { \frac { \mathrm { d } y } { \mathrm { d } x } + \frac { \mathrm { d } x } { \mathrm { d } x } + } \end{array}$ $\begin{array} { r } { \frac { \mathrm { d } } { \mathrm { d } x } ( 5 ) = 0 \Rightarrow \frac { \mathrm { d } y } { \mathrm { d } x } + 1 + 0 = 0 } \end{array}$ . Solving for $\frac { \mathrm { d } y } { \mathrm { d } x }$ gives $\textstyle { \frac { \mathrm { d } y } { \mathrm { d } x } } = - 1$

Assume the LL cost is at least twice diferentiable w.r.t. both y and $^ { x , }$ then we have $\begin{array} { r } { \frac { \partial \mathcal { L } ( \pmb { x } ^ { * } ( \pmb { y } ) , \pmb { y } ) } { \partial \pmb { x } ^ { * } ( \pmb { y } ) } = 0 } \end{array}$ due to the optimality condition where $\pmb { x } ^ { * }$ is a stationary point. Derive the equation $\begin{array} { r } { \frac { \partial \mathcal { L } ( \pmb { x } ^ { * } ( \pmb { y } ) , \pmb { y } ) } { \partial \pmb { x } ^ { * } ( \pmb { y } ) } = 0 } \end{array}$ on both sides w.r.t. y giving us

$$
\frac {\partial^ {2} \mathcal {L} (\boldsymbol {x} ^ {*} (\boldsymbol {y}) , \boldsymbol {y})}{\partial \boldsymbol {x} ^ {*} (\boldsymbol {y}) \partial \boldsymbol {y}} + \frac {\partial^ {2} \mathcal {L} (\boldsymbol {x} ^ {*} (\boldsymbol {y}) , \boldsymbol {y})}{\partial \boldsymbol {x} ^ {*} (\boldsymbol {y}) \partial \boldsymbol {x} ^ {*} (\boldsymbol {y})} \cdot \frac {\partial \boldsymbol {x} ^ {*} (\boldsymbol {y})}{\partial \boldsymbol {y}} = 0.\tag{4.11}
$$

This leads to the indirect gradient $\frac { \partial \pmb { x } ^ { * } ( \pmb { y } ) } { \partial \pmb { y } }$ as

$$
\frac {\partial \boldsymbol {x} ^ {*} (\boldsymbol {y})}{\partial \boldsymbol {y}} = - \left(\frac {\partial^ {2} \mathcal {L} (\boldsymbol {x} ^ {*} (\boldsymbol {y}) , \boldsymbol {y})}{\partial \boldsymbol {x} ^ {*} (\boldsymbol {y}) \partial \boldsymbol {x} ^ {*} (\boldsymbol {y})}\right) ^ {- 1} \frac {\partial^ {2} \mathcal {L} (\boldsymbol {x} ^ {*} (\boldsymbol {y}) , \boldsymbol {y})}{\partial \boldsymbol {x} ^ {*} (\boldsymbol {y}) \partial \boldsymbol {y}},\tag{4.12}
$$

The strength of (4.12) is that we convert the indirect gradient among the variables y and x to direct gradients of $\mathcal { L }$ at the cost of an inversion of a Hessian matrix. However, the weakness is that a Hessian is often too large to compute, thus it is common to solve a linear system leveraging the fast Hessian-vector product.

Example 4.3 (Cost of Storing and Inverting the Hessian) Assume both UL and LU costs involve a network with merely 1 million $( 1 0 ^ { 6 } )$ parameters (32-bit float numbers), thus each network only needs a space of $1 0 ^ { 6 } \times 4 \mathrm { B y t e } = 4 \mathrm { M } B$ to store, while their Hessian matrix needs a space of $( 1 0 ^ { 6 } ) ^ { 2 } \times 4 \mathrm { B y t e } = 4 \mathrm { T } B$ to store. This indicates that a Hessian matrix cannot even be explicitly stored in the memory of a low-power computer, thus directly calculating its inverse is impractical.

Recall that our goal is to compute the gradient in (4.3), substituting (4.12) into (4.3) gives us:

$$
\begin{array}{l} \nabla_ {\boldsymbol {y}} \mathcal {U} = \frac {\partial \mathcal {U} (\boldsymbol {y} , \boldsymbol {x} ^ {*})}{\partial \boldsymbol {y}} - \underbrace {\frac {\partial \mathcal {U} (\boldsymbol {y} , \boldsymbol {x} ^ {*})}{\partial \boldsymbol {x} ^ {*}}} _ {\boldsymbol {v} ^ {\top}} \underbrace {\left(\frac {\partial^ {2} \mathcal {L} (\boldsymbol {x} ^ {*} (\boldsymbol {y}) , \boldsymbol {y})}{\partial \boldsymbol {x} ^ {*} (\boldsymbol {y}) \partial \boldsymbol {x} ^ {*} (\boldsymbol {y})}\right) ^ {- 1}} _ {(\boldsymbol {H} ^ {\top}) ^ {- 1}} \frac {\partial^ {2} \mathcal {L} (\boldsymbol {x} ^ {*} (\boldsymbol {y}) , \boldsymbol {y})}{\partial \boldsymbol {x} ^ {*} (\boldsymbol {y}) \partial \boldsymbol {y}} \\ = \frac {\partial \mathcal {U} (\boldsymbol {y} , \boldsymbol {x} ^ {*})}{\partial \boldsymbol {y}} - \boldsymbol {q} ^ {\top} \cdot \frac {\partial^ {2} \mathcal {L} (\boldsymbol {x} ^ {*} (\boldsymbol {y}) , \boldsymbol {y})}{\partial \boldsymbol {x} ^ {*} (\boldsymbol {y}) \partial \boldsymbol {y}} \end{array} .\tag{4.13}
$$

Then we can solve the linear system $H q = v$ for $\pmb q ^ { \top }$ by optimizing

$$
\boldsymbol {q} ^ {*} = \min \arg_ {\boldsymbol {q}} Q (\boldsymbol {q}) = \min \arg_ {\boldsymbol {q}} \frac {1}{2} \boldsymbol {q} ^ {\top} \boldsymbol {H} \boldsymbol {q} - \boldsymbol {q} ^ {\top} \boldsymbol {v},\tag{4.14}
$$

using eficient linear solvers such as a simple gradient descent or conjugate gradient method [459]. For gradient descent, we need to compute the gradient of $Q$ as $\begin{array} { r } { \dot { \frac { \partial Q ( q ) } { \partial q } } = } \end{array}$ $H q - v$ , where $_ { H q }$ can be computed using the fast Hessian-vector product, i.e., a Hessian-vector product is the gradient of a gradient-vector product:

$$
\pmb {H} \pmb {q} = \frac {\partial^ {2} \mathcal {L}}{\partial \pmb {x} \partial \pmb {x}} \cdot \pmb {q} = \frac {\partial (\frac {\partial \mathcal {L}}{\partial \pmb {x}} \cdot \pmb {q})}{\partial \pmb {x}},\tag{4.15}
$$

where $\textstyle { \frac { \partial { \mathcal { L } } } { \partial { \boldsymbol { x } } } } \cdot { \boldsymbol { q } }$ is a scalar. This means that the Hessian matrix H is not explicitly computed or stored. We summarize the computation of implicit diferentiation with linear systems in Algorithm 2. The algorithm using a conjugate gradient is similar.

<div class="mineru-algorithm" style="white-space: pre-wrap; font-family:monospace;">
Algorithm 2 Computing Implicit Differentiation via Linear System.

1: Input: The current UL variable y and the optimal LL variable  $x^{*}$ .
2: Initialization: k = 1, learning rate  $\eta$ .
3: while Not Convergent ( $\|q_{k}-q_{k-1}\|$  is large enough) do
4: Perform gradient descent:
 $q_{k}=q_{k-1}-\eta(Hq_{k-1}-v)$ , (4.16)
where  $Hq_{k-1}$  is computed via the fast Hessian-vector product.
5: end while
6: Assign  $q=q_{k}$ 
7: Compute  $\nabla_{y}U$  in (4.3) as:
 $\nabla_{y}U=\frac{\partial U(y,x^{*})}{\partial y}-\underbrace{\left(\frac{\partial^{2}L(x^{*}(y),y)}{\partial y\partial x^{*}(y)}\cdot q\right)^{\mathsf{T}}} _{(H_{yx}\cdot q)^{\mathsf{T}}}$ , (4.17)
where  $H_{yx}\cdot q$  can also be computed efficiently using the Hessian-vector product.
</div>

Approximations. Implicit diferentiation is complicated to implement but there is one approximation, which is to ignore the implicit components and only use the direct part $\begin{array} { r } { \hat { \nabla } _ { \pmb { y } } U \approx \frac { \partial U } { \partial \pmb { y } } \Big | _ { \pmb { x } _ { T } } } \end{array}$ . This is equivalent to taking the solution $\mathbf { \nabla } _ { \mathbf { \mathcal { X } } \mathcal { T } }$ from the LL optimization as constants in the UL problem. Such an approximation is more eficient but introduces an error term

$$
\epsilon \sim \left| \frac {\partial U}{\partial \pmb {x} ^ {*}} \frac {\partial \pmb {x} ^ {*}}{\partial \pmb {y}} \right|.\tag{4.18}
$$

Nevertheless, it is useful when the implicit gradients contain products of small second-order derivatives, which depends on the specific NLS problems.

## 4.3 Diferentiation on Manifold

Given that the state of a typical SLAM system is bound to evolve on a manifold, optimization on manifolds plays a crucial role in solving back-end SLAM problems. We next derive the Jacobians required to diferentiate with respect to variables belonging to Lie groups, which is an essential step for diferentiation on the manifold.

## 4.3.1 Lie Group Derivatives

Since we introduced the basic concepts of Lie group, Lie algebra, and their basic operations $( e . g .$ , exponential and logarithmic maps) in Chapter 2, this section will briefly recap those concepts but mainly focus on the definition of their derivatives, which is essential for solving a diferentiable optimization problem.

Consider a Lie group’s manifold . Each point $x$ on this smooth manifold possesses a unique tangent space, denoted by $T _ { x } { \mathcal { M } }$ , where the fundamental principles of calculus are valid. The Lie algebra, represented as m, is a vector space that can be locally defined to the point χ as m $= T _ { x } { \mathcal { M } }$ . The exponential map exp : ${ \mathfrak { m } } \to { \mathcal { M } }$ projects elements from the Lie algebra to the Lie group, while the logarithmic map log : $\mathcal { M } $ m serves as its inverse, establishing a bi-directional relationship:

$$
\boldsymbol {\chi} = \exp (\boldsymbol {\tau} ^ {\wedge}) \Leftrightarrow \boldsymbol {\tau} ^ {\wedge} = \log (\boldsymbol {\chi}),\tag{4.19}
$$

where hat $\wedge$ is a linear invertible map, and $\tau ^ { \wedge } \in \mathfrak { m }$ . By representing the coordinates within the Lie algebra as vectors $\tau$ in $\mathbb { R } ^ { n }$ , we can define mappings between vector $\tau$ and the Lie group χ:

$$
\boldsymbol {\chi} = \operatorname{Exp} (\boldsymbol {\tau}) \Leftrightarrow \boldsymbol {\tau} = \operatorname{Log} (\boldsymbol {\chi}),\tag{4.20}
$$

where we redefined the exponential and logarithm maps to directly use a vector as input and output, respectively.

To calculate derivatives on Lie groups, it is crucial to first understand the relative change between two manifold elements, say $x _ { 1 }$ and $x _ { 2 }$ . These changes are quantified by first defining the  and  operators, which combine one exponential/logarithmic map and one composition. Because composition is generally non-commutative, these operators have right- and left- versions, depending on the order of the operands. The right operators are:

$$
\begin{array}{l l} \text {right-} \oplus : & \boldsymbol {\chi} _ {2} = \boldsymbol {\chi} _ {1} \oplus \boldsymbol {\tau} \triangleq \boldsymbol {\chi} _ {1} \circ \operatorname{Exp} \left(\boldsymbol {\tau}\right), \\ \text {right-} \ominus : & \boldsymbol {\tau} = \boldsymbol {\chi} _ {2} \ominus \boldsymbol {\chi} _ {1} \triangleq \operatorname{Log} \left(\boldsymbol {\chi} _ {1} ^ {- 1} \circ \boldsymbol {\chi} _ {2}\right). \end{array}\tag{4.21}
$$

The placement of $\tau$ on the right-hand side in (4.21) signifies that it is expressed in the local frame at $x _ { 1 }$ . Conversely, the left operators in (4.22) reflect a global frame perspective:

$$
\begin{array}{l l} \text {left-} \oplus : & \boldsymbol {\chi} _ {2} = \boldsymbol {\varepsilon} \oplus \boldsymbol {\chi} _ {1} \triangleq \operatorname{Exp} \left(\boldsymbol {\varepsilon}\right) \circ \boldsymbol {\chi} _ {1}, \\ \text {left-} \ominus : & \boldsymbol {\varepsilon} = \boldsymbol {\chi} _ {2} \ominus \boldsymbol {\chi} _ {1} \triangleq \operatorname{Log} \left(\boldsymbol {\chi} _ {2} \circ \boldsymbol {\chi} _ {1} ^ {- 1}\right), \end{array}\tag{4.22}
$$

where $\varepsilon$ is expressed in the global frame. Both τ and ε can be viewed as incremental perturbations to the manifold elements. By using corresponding composition operators $\oplus$ and , the variations are expressed as vectors in the tangent space.

With the right and $\ominus$ operators in place, we use the Jacobian matrix J to describe perturbations on manifolds. The Jacobian captures the essence of infinitesimal perturbations τ within the tangent space m:

$$
\begin{array}{l} \frac {\partial f (\boldsymbol {\chi})}{\partial \boldsymbol {\chi}} \triangleq \lim _ {\tau \to 0} \frac {f (\boldsymbol {\chi} \oplus \boldsymbol {\tau}) \ominus f (\boldsymbol {\chi})}{\boldsymbol {\tau}} \\ = \lim _ {\tau \to 0} \frac {f (\boldsymbol {\chi} \circ \operatorname{Exp} (\boldsymbol {\tau})) \ominus f (\boldsymbol {\chi})}{\boldsymbol {\tau}} \\ = \lim _ {\boldsymbol {\tau} \to 0} \frac {\operatorname{Log} \left(f (\boldsymbol {\chi}) ^ {- 1} \circ f (\boldsymbol {\chi} \circ \operatorname{Exp} (\boldsymbol {\tau}))\right)}{\boldsymbol {\tau}}. \end{array}\tag{4.23}
$$

Let $g ( \pmb { \tau } ) = \mathrm { L o g } \left( f ( \pmb { \chi } ) ^ { - 1 } \circ f ( \pmb { \chi } \circ \mathrm { E x p } \left( \pmb { \tau } \right) ) \right)$ , then the right Jacobian $\scriptstyle J _ { R }$ can be expressed as the derivative of $g ( \tau )$ at $\tau = 0$

$$
\frac {\partial f (\boldsymbol {\chi})}{\partial \boldsymbol {\chi}} = \boldsymbol {J} _ {R} = \left. \frac {\partial g (\boldsymbol {\tau})}{\partial \boldsymbol {\tau}} \right| _ {\boldsymbol {\tau} = 0}.\tag{4.24}
$$

In this way, the derivatives of $f ( \chi )$ with respect to $x$ in the manifold are represented by the Jacobian matrix $J _ { R } \in \mathbb { R } ^ { m \times n }$ , where m and n are the dimensions of the Lie groups and ${ \mathcal { N } } .$ , respectively. The right Jacobian matrix performs a linear mapping from the tangent space m to the tangent space $\mathfrak { n } = T _ { f ( \mathfrak { x } ) } \mathcal { N }$

Similarly, consider an infinitesimal perturbation $\varepsilon \in T _ { g } \mathcal { M }$ applied to the Lie group element $x \mathrm { . }$ , the left Jacobian $J _ { L }$ can be defined with the left plus and minus operators:

$$
\begin{array}{r l} \frac {\partial f (\boldsymbol {\chi})}{\partial \boldsymbol {\chi}} & \triangleq \lim _ {\varepsilon \to 0} \frac {f (\boldsymbol {\varepsilon} \oplus \boldsymbol {\chi}) \ominus f (\boldsymbol {\chi})}{\varepsilon} \\ & = \lim _ {\varepsilon \to 0} \frac {f (\mathrm{Exp} (\boldsymbol {\varepsilon}) \circ \boldsymbol {\chi}) \ominus f (\boldsymbol {\chi})}{\varepsilon} \\ & = \lim _ {\varepsilon \to 0} \frac {\mathrm{Log} \left(f (\mathrm{Exp} (\boldsymbol {\varepsilon}) \circ \boldsymbol {\chi}) \circ f (\boldsymbol {\chi}) ^ {- 1}\right)}{\varepsilon} \\ & = \frac {\partial \mathrm{Log} \left(f (\mathrm{Exp} (\boldsymbol {\varepsilon}) \circ \boldsymbol {\chi}) \circ f (\boldsymbol {\chi}) ^ {- 1}\right)}{\partial \boldsymbol {\varepsilon}} \bigg | _ {\boldsymbol {\varepsilon} = 0}. \end{array}\tag{4.25}
$$

The resulting left Jacobian $\ b { J } _ { L } \in \mathbb { R } ^ { n \times m }$ is also a linear mapping, but in the global tangent space from $T _ { g } \mathcal { M }$ to $T _ { g } \mathcal { N }$

To delve into the local perturbations around a point $x _ { 1 }$ , we consider perturbations τ as $\pmb { \tau } = \pmb { \chi } \ominus \pmb { \chi } _ { 1 }$ , with $x$ being a perturbed version of $x _ { 1 }$ . The covariance matrices $\Sigma _ { x }$ defined on the tangent space are derived using the expectation operator $\mathbb { E } ,$ enabling the representation of uncertainties and their propagation:

$$
\boldsymbol {\Sigma} _ {\boldsymbol {\chi}} \triangleq \mathbb {E} [ \boldsymbol {\tau} \boldsymbol {\tau} ^ {\top} ] = \mathbb {E} [ (\boldsymbol {\chi} \ominus \boldsymbol {\chi} _ {1}) (\boldsymbol {\chi} \ominus \boldsymbol {\chi} _ {1}) ^ {\top} ].\tag{4.26}
$$

These covariance matrices facilitate the establishment of Gaussian distributions on the manifold, expressed as $\pmb { \chi } \sim \mathcal { N } ( \pmb { \chi } _ { 1 } , \pmb { \Sigma } _ { \pmb { \chi } } )$ . It is important to note that the covariance matrices $\Sigma _ { x }$ are defined on the tangent space $T _ { x _ { 1 } } { \mathcal { M } } ,$ , which allows the uncertainty in the manifold to be represented by a vector and be propagated in the form of covariance matrices.

Example 4.4 (Visual-Inertial Rotation Estimation) Consider a robot equipped with an inertial measurement unit (IMU) and a camera. Given noisy observations $R _ { \mathrm { I M U } }$ and $R _ { \mathrm { { C a m } } }$ from both sensors, the orientation of the robot can be estimated by minimizing the discrepancy between the measurements, which can be formulated as a nonlinear least squares problem on the manifold SO(3):

$$
\hat {\boldsymbol {R}} = \arg \min _ {\boldsymbol {R} \in \mathrm{SO(3)}} f (\boldsymbol {R}, \boldsymbol {R} _ {\mathrm{IMU}}, \boldsymbol {R} _ {\mathrm{Cam}}),\tag{4.27}
$$

where $f ( \cdot )$ is the cost function that quantifies the diferences between the estimated orientation R and the sensor measurements $R _ { \mathrm { I M U } }$ and $R _ { \mathrm { { C a m } } }$ . With the Jacobian matrices in place, the optimization on the manifold $\mathrm { S O ( 3 ) }$ ) for the pose estimation can be efectively managed. The cost function $f ( \cdot )$ can be written as:

$$
f (\boldsymbol {R}) = \left\| \operatorname{Log} \left(\boldsymbol {R} _ {\mathrm{IMU}} ^ {- 1} \boldsymbol {R}\right) \right\| ^ {2} + \left\| \operatorname{Log} \left(\boldsymbol {R} _ {\mathrm{Cam}} ^ {- 1} \boldsymbol {R}\right) \right\| ^ {2}.\tag{4.28}
$$

To minimize $f ( R )$ , we need to compute its gradient with respect to R on the manifold $\mathrm { S O ( 3 ) }$ . The gradient can be derived using the right Jacobian $J _ { R }$ as:

$$
\begin{array}{c} \nabla f (\boldsymbol {R}) = 2 \left(\frac {\partial \mathrm{Log} \left(\boldsymbol {R} _ {\mathrm{IMU}} ^ {- 1} \boldsymbol {R}\right)}{\partial \boldsymbol {R}}\right) ^ {\top} \mathrm{Log} \left(\boldsymbol {R} _ {\mathrm{IMU}} ^ {- 1} \boldsymbol {R}\right) \\ + 2 \left(\frac {\partial \mathrm{Log} \left(\boldsymbol {R} _ {\mathrm{Cam}} ^ {- 1} \boldsymbol {R}\right)}{\partial \boldsymbol {R}}\right) ^ {\top} \mathrm{Log} \left(\boldsymbol {R} _ {\mathrm{Cam}} ^ {- 1} \boldsymbol {R}\right). \end{array}\tag{4.29}
$$

The gradient $\nabla f ( R )$ can be used in conjunction with optimization algorithms like gradient descent which moves along the tangent space and reprojecting back to the manifold to update the pose R iteratively:

$$
\boldsymbol {R} _ {k + 1} = \boldsymbol {R} _ {k} \mathrm{Exp} \left(- \alpha \nabla f (\boldsymbol {R})\right),\tag{4.30}
$$

where α is the step size. This iterative process continues until the cost function $f ( R )$ converges to a minimum, providing a pose estimate $\hat { R }$ that leverages both sensor measurements.

## 4.3.2 Diferentiation Operations on Manifold

For typical manifold operations, we can derive closed-form expressions for the Jacobians associated with inversion, composition, and group actions. These expressions facilitate a comprehensive approach to optimization in SLAM, by enabling the computation of function derivatives on manifolds with the chain rule:

$$
\frac {\partial \mathcal {Z}}{\partial \boldsymbol {\chi}} = \frac {\partial \mathcal {Z}}{\partial \boldsymbol {\mathcal {Y}}} \frac {\partial \boldsymbol {\mathcal {Y}}}{\partial \boldsymbol {\chi}},\tag{4.31}
$$

where $\mathcal { Z } = g ( \mathcal { Y } )$ , and $\mathcal { V } = f ( \boldsymbol { \chi } )$

Jacobians of inversion can be derived through the application of the function $f ( \chi ) = \chi ^ { - 1 }$ with (4.23) for the right Jacobian $\scriptstyle J _ { R }$ , which leads to:

$$
\begin{array}{l} \frac {\partial \boldsymbol {\chi} ^ {- 1}}{\partial \boldsymbol {\chi}} \triangleq \lim _ {\boldsymbol {\tau} \to 0} \frac {\operatorname{Log} \left((\boldsymbol {\chi} ^ {- 1}) ^ {- 1} (\boldsymbol {\chi} \operatorname{Exp} (\boldsymbol {\tau})) ^ {- 1}\right)}{\boldsymbol {\tau}} \\ = \lim _ {\boldsymbol {\tau} \to 0} \frac {\operatorname{Log} \left(\boldsymbol {\chi} \operatorname{Exp} (\boldsymbol {\tau}) ^ {- 1} \boldsymbol {\chi} ^ {- 1}\right)}{\boldsymbol {\tau}} \\ = \lim _ {\boldsymbol {\tau} \to 0} \frac {(\boldsymbol {\chi} (- \boldsymbol {\tau}) ^ {\wedge} \boldsymbol {\chi} ^ {- 1}) ^ {\vee}}{\boldsymbol {\tau}}. \end{array}\tag{4.32}
$$

Jacobians of composition can be derived through the application of the function $f ( \chi ) = \chi \circ \chi _ { 1 }$ with the Equation (4.23). The derivative of the composition operator $x \circ x _ { 1 }$ with respect to $x$ is:

$$
\begin{array}{l} \frac {\partial (\boldsymbol {\chi} \circ \boldsymbol {\chi} _ {1})}{\partial \boldsymbol {\chi}} \triangleq \lim _ {\tau \to 0} \frac {\operatorname{Log} \left((\boldsymbol {\chi} \boldsymbol {\chi} _ {1}) ^ {- 1} (\boldsymbol {\chi} \operatorname{Exp} (\tau) \boldsymbol {\chi} _ {1})\right)}{\tau} \\ = \lim _ {\tau \to 0} \frac {\operatorname{Log} \left(\boldsymbol {\chi} _ {1} ^ {- 1} \operatorname{Exp} (\tau) \boldsymbol {\chi} _ {1}\right)}{\tau} \\ = \lim _ {\tau \to 0} \frac {(\boldsymbol {\chi} _ {1} ^ {- 1} \boldsymbol {\tau} ^ {\wedge} \boldsymbol {\chi} _ {1}) ^ {\vee}}{\tau}. \end{array}\tag{4.33}
$$

The derivative of the composition operator $x \circ x _ { 1 }$ with respect to $x _ { 1 }$ is:

$$
\begin{array}{l} \frac {\partial (\boldsymbol {\chi} \circ \boldsymbol {\chi} _ {1})}{\partial \boldsymbol {\chi} _ {1}} \triangleq \lim _ {\tau \to 0} \frac {\operatorname{Log} \left((\boldsymbol {\chi} \boldsymbol {\chi} _ {1}) ^ {- 1} (\boldsymbol {\chi} \boldsymbol {\chi} _ {1} \operatorname{Exp} (\boldsymbol {\tau}))\right)}{\boldsymbol {\tau}} \\ = \lim _ {\tau \to 0} \frac {\operatorname{Log} \left(\operatorname{Exp} (\boldsymbol {\tau})\right)}{\boldsymbol {\tau}} \\ = \boldsymbol {I}. \end{array}\tag{4.34}
$$

Jacobians of the manifold are characterized by the right Jacobian of $x$ which is derived from the exponential map of $\tau \in \mathbb { R } ^ { m }$ . This is expressed as:

$$
\boldsymbol {J} _ {r} (\boldsymbol {\tau}) \triangleq \frac {\boldsymbol {\tau} \partial \mathrm{Exp} (\boldsymbol {\tau})}{\partial \boldsymbol {\tau}}.\tag{4.35}
$$

The right Jacobian conveys minor changes in $\tau$ to modifications in the local tangent space at Exp (τ). Similarly, the left Jacobian of $x$ maps changes of $\tau$ to variations within the global tangent space of the manifold. This is expressed as:

$$
\boldsymbol {J} _ {l} (\boldsymbol {\tau}) \triangleq \frac {\varepsilon \partial \mathrm{Exp} (\boldsymbol {\tau})}{\partial \boldsymbol {\tau}}.\tag{4.36}
$$

Jacobians of group action depends on the specific group action set $v \in \mathcal V$ The group action is defined as:

$$
\begin{array}{l} J _ {\chi} ^ {\chi \cdot v} \triangleq \frac {x D \chi \cdot v}{D \chi}, \\ J _ {v} ^ {\chi \cdot v} \triangleq \frac {{} ^ {v} D \chi \cdot v}{D v}, \end{array}\tag{4.37}
$$

where $x \in \mathcal { M }$ and $v \in \mathcal V .$

Example 4.5 (Robot Arm) Consider a robotic arm with two joints, $\scriptstyle { R _ { 1 } }$ and $R _ { 2 }$ , each represented by an element in SO(3). The final orientation of the robot’s end-efector is determined by the composition of the joint rotations:

$$
\pmb {R} = \pmb {R} _ {1} \circ \pmb {R} _ {2}.\tag{4.38}
$$

To evaluate the impact of small perturbations τ in $\pmb { R } _ { 1 }$ and $R _ { 2 }$ on the end-efector orientation R. It can be quantified using the Jacobians of composition:

$$
\begin{array}{l} \frac {\partial (\boldsymbol {R} _ {1} \circ \boldsymbol {R} _ {2})}{\partial \boldsymbol {R} _ {1}} = \lim _ {\tau \to 0} \frac {(\boldsymbol {R} _ {2} ^ {- 1} \boldsymbol {\tau} ^ {\wedge} \boldsymbol {R} _ {2}) ^ {\vee}}{\tau}, \\ \frac {\partial (\boldsymbol {R} _ {1} \circ \boldsymbol {R} _ {2})}{\partial \boldsymbol {R} _ {2}} = \boldsymbol {I}. \end{array}\tag{4.39}
$$

This example implies that adjustments to the first joint $\pmb { R } _ { 1 }$ afect the final orientation R through a transformation influenced by the current state of the second joint $R _ { 2 }$ . However, changes in the second joint $R _ { 2 }$ directly impact R without being influenced by the first joint $R _ { 1 }$

## 4.4 Numerical Challenges of Automatic Diferentiation and Modern Libraries

AutoDif is a cornerstone technique for computing derivatives accurately and efficiently in various optimization contexts, including diferentiation on manifolds. Diferentiation on manifolds poses unique challenges due to the complex geometrical properties inherent in manifold structures, which can afect the performance and applicability of AutoDif. In diferential optimization, these challenges become pronounced as AutoDif interacts with the curved space of manifolds, potentially introducing numerical instability and inaccuracies.

This section provides examples of numerical issues that arise when using automatic diferentiation for manifold-based optimization tasks. Particular attention is paid to the complexities involved in maintaining numerical stability and precision in the presence of manifold constraints, such as those found in constrained optimization and in systems defined by diferential equations on manifolds. We will take the $\mathrm { P y }$ Pose library [1145] as an example, which defines a general data structure, LieTensor for Lie Group and Lie Algebra. Specifically, we will show some numerical challenges and how PyPose tackles these challenges.

Example 4.6 (Exponential Mapping and Quaternions) The exponential map is a fundamental concept in the theory of Lie groups and is particularly critical when transitioning between Lie algebras and Lie groups represented by quaternions. This mapping enables the translation of angular velocities from the algebraic structure in $\mathbb { R } ^ { 3 }$ to rotational orientations in the group of unit quaternions $\mathbb { S } ^ { 3 }$ . Analytically, the exponential map for quaternions is derived from the Rodrigues’ rotation formula, which relates a vector in $\mathbb { R } ^ { 3 }$ to the corresponding rotation. Given a vector x in $\mathbb { R } ^ { 3 }$ , representing the axis of rotation scaled by the rotation angle, the quaternion representation of the rotation is given by:

$$
\operatorname{Exp} (\boldsymbol {\nu}) = \left[ \sin \left(\frac {\| \boldsymbol {\nu} \|}{2}\right) \frac {\boldsymbol {\nu} ^ {\top}}{\| \boldsymbol {\nu} \|}, \cos \left(\frac {\| \boldsymbol {x} \|}{2}\right) \right] ^ {\top}\tag{4.40}
$$

where $\| \nu \|$ represents the magnitude of $\nu ,$ corresponding to the angle of rotation, and $\frac { \nu } { \left\| \nu \right\| }$ is the unit vector in the direction of $\pmb { \nu } .$ .

One of the challenges of implementing a diferentiable LieTensor is that one often need to calculate numerically problematic terms such as $\frac { \sin \nu } { \nu }$ in (4.40) for the Exp and Log mapping [1083]. The direct computation of sine and cosine functions for very small angles can lead to precision issues due to the finite representation of floating-point numbers in computer systems. To manage these issues and maintain numerical stability, $\mathrm { P y }$ Pose takes the Taylor expansion to avoid calculating the division by zero.

$$
\operatorname{Exp} (\boldsymbol {\nu}) = \left\{ \begin{array}{l l} \left[ \boldsymbol {\nu} ^ {T} \gamma_ {e}, \cos (\frac {\| \boldsymbol {\nu} \|}{2}) \right] ^ {T} & \text {if \| \boldsymbol{\nu}\| > eps} \\ \left[ \boldsymbol {\nu} ^ {T} \gamma_ {o}, 1 - \frac {\| \boldsymbol {\nu} \| ^ {2}}{8} + \frac {\| \boldsymbol {\nu} \| ^ {4}}{3 8 4} \right] ^ {T} & \text {otherwise,} \end{array} \right.\tag{4.41}
$$

where $\begin{array} { r } { \gamma _ { e } = \frac { \sin ( \frac { \| \pmb { \nu } \| } { 2 } ) } { \| \pmb { \nu } \| } } \end{array}$ when $\| \nu \|$ is significant, and $\begin{array} { r } { \gamma _ { o } = \frac { 1 } { 2 } - \frac { \Vert \pmb { \nu } \Vert ^ { 2 } } { 4 8 } + \frac { \Vert \pmb { \nu } \Vert ^ { 4 } } { 3 8 4 0 } } \end{array}$ for small $\lVert \nu \rVert$ ensuring precise calculations across all ranges of rotation magnitudes. Here, eps is the smallest machine number where $1 + \mathrm { e p s } \neq 1$ . This analytical-to-numerical progression demonstrates the importance of accurate and stable methods for computing exponential maps in applications that require high-fidelity rotation representations.

LieTensor is diferent from the existing libraries in several aspects: (1) $\mathrm { P y }$ Pose supports AutoDif for any order gradient and is compatible with most popular devices, such as CPU, GPU, TPU, and Apple silicon GPU, while other libraries like LieTorch [1083] implement customized CUDA kernels and only support $1 ^ { \mathrm { s t } } -$ order gradient. (2) LieTensor supports parallel computing of gradient with the vmap operator, which allows it to compute Jacobian matrices much faster. (3) Libraries such as LieTorch, JaxLie [1244], and Theseus only support Lie groups, while PyPose supports both Lie groups and Lie algebras. As a result, one can directly call the Exp and Log maps from a LieTensor instance, which is more flexible and user-friendly. Moreover, the gradient with respect to both types can be automatically calculated and back-propagated. The readers may find a list of supported LieTensor operations in [1144] and the tutorial of PyPose is available in [1147]. The usages of a LieTensor and its automatic diferentiation can be found at https: //github.com/pypose/slambook-snippets/blob/main/lietensor.ipynb.

## 4.4.1 Example of Implementation of Diferentiable Optimization

To enable end-to-end learning with bilevel optimization, one needs to integrate general optimizers beyond gradient-based methods such as SGD [930] and Adam [579] required by neural methods, since many problems in SLAM require other optimizations algorithms such as constrained or $2 ^ { \mathrm { n d } }$ -order optimization [51]. Moreover, practical problems have outliers, hence one needs to robustify the loss as described in Chapter 3. Next we consider an IRLS approach to SLAM as introduced in Section 3.3, and present the intuition behind the optimization-oriented interfaces of PyPose, including solver, kernel, corrector, and strategy for using the $2 ^ { \mathrm { n d } } .$ order Levenberg-Marquardt (LM) optimizer.

Let us start by considering a weighted least square problem:

$$
\min _ {\boldsymbol {y}} \sum_ {i} \left(\boldsymbol {h} _ {i} (\boldsymbol {x} _ {i}) - \boldsymbol {z} _ {i}\right) ^ {T} \boldsymbol {\Sigma} _ {i} \left(\boldsymbol {h} _ {i} (\boldsymbol {x} _ {i}) - \boldsymbol {z} _ {i}\right),\tag{4.42}
$$

where $h ( \cdot )$ is a regression model (Module), $\ b { x } \in \mathbb { R } ^ { n }$ is the parameters to be optimized, $\boldsymbol { h } _ { i }$ denotes prediction for the i-th input sample, $\pmb { \Sigma } _ { i } \in \mathbb { R } ^ { d \times d }$ is a square information matrix. The solution to (4.42) of an LM algorithm is computed by iteratively updating an estimate $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { t } }$ via $\pmb { x } _ { t } \gets \pmb { x } _ { t - 1 } + \delta _ { t }$ , where the update step $\delta _ { t }$ is computed as:

$$
\sum_ {i} \left(\boldsymbol {\Lambda} _ {i} + \lambda \cdot \operatorname{diag} \left(\boldsymbol {\Lambda} _ {i}\right)\right) \boldsymbol {\delta} _ {t} = - \sum_ {i} \boldsymbol {J} _ {i} ^ {T} \boldsymbol {\Sigma} _ {i} \boldsymbol {r} _ {i},\tag{4.43}
$$

where ${ \pmb r } _ { i } = { \pmb h } _ { i } ( { \pmb x } _ { i } ) - z _ { i }$ is the i-th residual error, $J _ { i }$ is the Jacobian of h computed at $\pmb { x } _ { t - 1 } , ~ \pmb { \Lambda } _ { i }$ is an approximated Hessian matrix computed as $\mathbf { \Lambda } \Lambda _ { i } = J _ { i } ^ { T } \pmb { \Sigma } _ { i } J _ { i }$ , and λ is a damping factor. To find step $\delta _ { t }$ , one needs a linear solver:

$$
\mathbf {A} \cdot \boldsymbol {\delta} _ {t} = \boldsymbol {\beta},\tag{4.44}
$$

where $\begin{array} { r } { \mathbf { A } = \sum _ { i } \big ( \mathbf { \boldsymbol { \Lambda } } _ { i } + \boldsymbol { \lambda } \cdot \mathrm { d i a g } ( \mathbf { \boldsymbol { \Lambda } } _ { i } ) \big ) , \boldsymbol { \beta } = - \sum _ { i } \mathbf { \boldsymbol { J } } _ { i } ^ { T } \Sigma _ { i } \boldsymbol { r } _ { i } } \end{array}$ . In practice, the square matrix A is often positive-definite, so we could leverage standard linear solvers such as Cholesky. If the Jacobian $J _ { i }$ is large and sparse, we may also use sparse solvers such as sparse Cholesky [161] or preconditioned conjugate gradient $\left( \mathrm { P C G } \right)$ [459] solver.

In practice, one often introduces robust kernel functions $\rho : \mathbb { R } \mapsto \mathbb { R }$ into (4.42) to reduce the efect of outliers:

$$
\min _ {\boldsymbol {y}} \sum_ {i} \rho \left(\boldsymbol {r} _ {i} ^ {T} \boldsymbol {\Sigma} _ {i} \boldsymbol {r} _ {i}\right),\tag{4.45}
$$

where $\rho$ is designed to down-weigh measurements with large residuals $\mathbf { \nabla } r _ { i } .$ . In this case, we need to adjust (4.43) to account for the presence of the robust kernel. A popular way is to use an IRLS method, Triggs’ correction [1107], which is also adopted by the Ceres [15] library. However, it needs $2 ^ { \mathrm { n d } }$ -order derivative of the kernel function $\rho ,$ which is always negative. This can lead $2 ^ { \mathrm { n d } }$ -order optimizers including LM to become unstable [1107]. Alternatively, $\mathrm { P y }$ Pose introduces an IRLS method, FastTriggs, which is faster yet more stable than Triggs by only involving the 1<sup>st</sup>-order derivative:

$$
\pmb {r} _ {i} ^ {\rho} = \sqrt {\rho^ {\prime} (c _ {i})} \pmb {r} _ {i}, \quad \pmb {J} _ {i} ^ {\rho} = \sqrt {\rho^ {\prime} (c _ {i})} \pmb {J} _ {i},\tag{4.46}
$$

where $c _ { i } = \boldsymbol { r } _ { i } ^ { T } \Sigma _ { i } \boldsymbol { r } _ { i } , \boldsymbol { r } _ { i } ^ { \rho }$ and $J _ { i } ^ { \rho }$ are the corrected model residual and Jacobian due to the introduction of kernel functions, respectively. More details about FastTriggs and its proof can be found in [1143], while IRLS was introduced in Section 3.3.

A simple LM optimizer may not converge to the global optimum if the initial guess is too far from the optimum. For this reason, we often need other strategies such as adaptive damping, dogleg, and trust region methods [697] to restrict each step, preventing it from stepping “too far”. To adopt those strategies, one may simply pass a strategy instance, e.g., TrustRegion, to an optimizer. In summary, PyPose supports easy extensions for the aforementioned algorithms by simply passing optimizer arguments to their constructor, including solver, strategy, kernel, and corrector. A list of available algorithms and examples can be found in [1146]. The usages of a 2nd-order optimization can be found at https: //github.com/pypose/slambook-snippets/blob/main/optimization.ipynb.

## 4.4.2 Related Open-source Libraries

Open-source libraries related to diferentiable optimization can be divided into three groups: (1) linear algebra, (2) machine learning libraries, and (3) specialized optimization libraries.

Linear Algebra Libraries are essential to machine learning and robotics research. NumPy [823], a linear algebra library for Python, ofers comprehensive operations on vectors and matrices while enjoying higher running speed due to its underlying well-optimized C code. Eigen [412], a high performance C++ linear algebra library, has been used in many projects such as TensorFlow [2], Ceres [15], GTSAM [252], and $\mathrm { g } ^ { 2 } \mathrm { o }$ [402]. ArrayFire [727], a GPU acceleration library for C, C++, Fortran, and $\mathrm { P y }$ thon, contains simple APIs and provides GPU-tuned functions.

Machine Learning Libraries focus more on operations on tensors (i.e., highdimensional matrices) and automatic diferentiation. Early machine learning frameworks, such as Torch [228], OpenNN [830], and MATLAB [738], provide primitive tools for researchers to develop neural networks. However, they only support CPU computation and lack concise APIs, which limit their usability. A few years later, deep learning frameworks such as Chainer [1101], Theano [24], and Cafe [518] arose to handle the increasing size and complexity of neural networks while supporting multi-GPU training with convenient APIs for users to build and train their neural networks. Furthermore, the recent frameworks, such as TensorFlow [2], PyTorch [850], and MXNet [184], provide a comprehensive and flexible ecosystem (e.g., APIs for multiple programming languages, distributed data parallel training, and facilitating tools for benchmark and deployment). Gvnn [430] introduced diferentiable transformation layers into Torch-based framework, leading to end-to-end geometric learning. JAX [110] can automatically diferentiate native Python and NumPy functions and is an extensible system for composable function transformations. In many ways, the existence of these frameworks facilitated and promoted the growth of deep learning. Recently, more eforts have been taken to combine standard optimization tools with deep learning. Recent work like Theseus [873] and Cvxpy-Layer [18] showed how to embed diferentiable optimization within deep neural networks. PyPose [1145] incorporates 2<sup>nd</sup>-order optimizers such as Gaussian-Newton and Levenberg-Marquardt and can compute any order gradients of Lie groups and Lie algebras, which are essential to robotics.

Other Specialized Optimization Libraries have been developed and leveraged in robotics. To mention a few, Ceres [15] is an open-source C++ library for large-scale nonlinear least squares optimization problems and has been widely used in SLAM. Pyomo [434] and JuMP [292] are optimization frameworks that have been widely used due to their flexibility in supporting a diverse set of tools for constructing, solving, and analyzing optimization models. CasADi [32] has been used to solve many real-world control problems in robotics due to its fast and effective implementations of diferent numerical methods for optimal control. Poseand factor-graph optimization also play an important role in robotics. For example, g<sup>2</sup>o [402] and GTSAM [252] are open-source C++ frameworks for graph-based nonlinear optimization, which provide concise APIs for constructing new problems and have been leveraged to solve several optimization problems in SLAM.

Optimization libraries have also been widely used in robotic control problems. To name a few, IPOPT [1139] is an open-source C++ solver for nonlinear programming problems based on interior-point methods and is widely used in robotics and control. Similarly, OpenOCL [590] supports a large class of optimization problems such as continuous time, discrete time, constrained, unconstrained, multi-phase, and trajectory optimization problems for real-time model-predictive control. Another library for large-scale optimal control and estimation problems is CT [383], which provides standard interfaces for diferent optimal control solvers and can be extended to a broad class of dynamical systems in robotic applications. Drake [1079] has solvers for common control problems and that can be directly integrated with its simulation tool boxes. Its system completeness made it favorable to researchers.

## 4.5 Further Readings & Recent Trends

Deep learning methods have witnessed significant development in recent years [1286]. As data-driven approaches, they are believed to perform better on visual tracking compared to traditional handcrafted features. Most studies on the subject employ end-to-end structures, including both supervised methods such DeepVO [1163] and TartanVO [1167] and unsupervised methods such as UnDeepVO [656] and Unsupervised VIO [1173]. It is generally observed that the supervised approaches achieve higher performance compared to their unsupervised counterparts since they can learn from a diverse range of ground truths such as pose, flow, and depth. Nevertheless, obtaining such ground truths in the real world is a labor-intensive process [1166].

Recently, hybrid methods have received increasing attention as they integrate the strengths of both geometry-based and deep-learning approaches. Several studies have explored the potential of integrating BA with deep learning methods to impose topological consistency between frames, such as attaching a BA layer to a learning network such as BA-Net [1068] and DROID-SLAM [1082]. Additionally, some works focused on compressing image features into codes (embedded features) and optimizing the pose-code graph during inference such as DeepFactors [236]. Furthermore, DifPoseNet [842] is proposed to predict poses and normal flows using networks and fine-tune the coarse predictions through a Cheirality layer. However, in these works, the learning-based methods and geometry-based optimization are decoupled and separately used in diferent sub-modules. The lack of integration between the front-end and back-end may result in sub-optimal performance. Besides, they only back-propagate the pose error “through” BA, thus the supervision is from the ground truth poses. In this case, BA is just a special layer of the network. Recently, iSLAM [346] connects the front-end and back-end bidirectionally and enforces the learning model to learn from geometric optimization through a bilevel optimization framework, which achieves performance improvement without external supervision. Some other tasks can also be formulated as bilevel optimization, e.g., reinforcement learning [1027], local planning [1216], global planning [186], feature matching [1267], and multi-robot routing [419].

Dense Map Representations

Victor Reijgwart, Jens Behley, Teresa Vidal-Calleja, Helen Oleynikova, Lionel Ott, Cyrill Stachniss and Ayoung Kim

We now shift our focus to a diferent aspect of SLAM, specifically its mapping component. The mapping problem is approached with the assumption that the robot’s pose has been estimated, and the objective is to construct a dense map of its surroundings. Indeed, typical approaches first solve for the robot trajectory using the SLAM back-end —as discussed in the previous chapters— and then reconstruct a dense map given the trajectory. In this chapter, we illustrate the details of the dense map representation, focusing on the map elements, data structures, and methods.

Early mapping approaches were predominantly based on sparse, landmark-based solutions as discussed in Chapter 1, which extract only a few salient features from the environment. However, the increase in compute capabilities paired with the advent of accurate 3D range sensors, such as mechanical 3D LiDARs or RGB-D cameras that provide detailed 3D range measurements at high frequencies, led to an increasing research interest in dense map representations. Dense maps are crucial for downstream tasks that require a detailed understanding of the environment, such as planning, navigation, manipulation and precise localization. This chapter explains how these dense methods leverage the full spectrum of range sensor data to refine and update comprehensive maps.

The chapter begins by presenting key sensor types that facilitate dense mapping, primarily focusing on range sensors that produce detailed range measurements, which are reviewed in Section 5.1. The chapter continues with an introduction to fundamental representation elements and data structures in Section 5.2, that we then tailor to specific applications in Section 5.4. Contrasting with sparse landmarkbased mapping, the choice of a dense map representation hinges on the sensor types used and the intended downstream applications. Key factors influencing the selection of the representation type are summarized in Section 5.5.

## 5.1 Range Sensing Preliminaries

Before we delve into dense map representations, we briefly summarize a key sensing modality, range sensors, often used for SLAM, providing the necessary context for the following discussion. Such sensors produce range measurements to the objects in the environment, including LiDAR sensors, time-of-flight (TOF) cameras, RGB-D cameras, and stereo cameras. Here, we concentrate on the most commonly used LiDAR sensors and RGB-D cameras that are predominately used in outdoor and indoor environments for SLAM and dense mapping.

![](images/397bf3ea6ff858f62e430dd83330be86e339a5e5f9cca5799262d9a8119765dd.jpg)  
Figure 5.1 Single ray and multi-ray types for LiDAR sensors. Sample data from real-world is visualized to show RGB, depth, and LiDAR range image.

## 5.1.1 Sensor Measurement Model

Let us start with a brief summary of the sensing mechanism and associated measurement model. In the case of LiDAR sensors,<sup>1</sup> the range measurements are generated using laser beams that are emitted, reflected by the environment, and then detected [935]. By measuring the time $t _ { \mathrm { e m i t } }$ when the laser beam is emitted and the time of detection $t _ { \mathrm { d e t e c t } }$ , we can derive the range r using the speed of light c as follows:

$$
r = \frac {c (t _ {\mathrm{detect}} - t _ {\mathrm{emit}})}{2}.\tag{5.1}
$$

A single ray measurement can be enhanced into a two-dimensional (2D) or threedimensional (3D) collection of points by employing an array of rays that move in a designated pattern, such as a 360-degree rotation or a specific shape. The collection of points generated by the sensor is referred to as point clouds, which serve as the fundamental element for creating maps. The LiDAR measurements can also be represented using a range image $\pmb { R } \in \mathbb { R } ^ { B \times M }$ , where the range of each of the B beams is stored for a single complete turn of the sensor, i.e., a complete 360◦ rotation. Thus, we have M measurements in the horizontal field of view of the sensor for each of the B beams; see illustrations and examples in Figure 5.1.

Other commonly used range sensors in robotics applications are RGB-D cameras, such as Microsoft’s Kinect and Azure Kinect DK, and Intel’s RealSense. RGB-D cameras provide —besides the RGB image $I _ { \mathrm { R G B } } \in \mathbb { R } ^ { 3 \times H \times W }$ of height H and width W— a depth map $\pmb { I } _ { D } \in \mathbb { R } ^ { H \times W }$ of the same dimension, where each pixel location contains the depth or range. To generate the depth map $I _ { D }$ , early RGB-D cameras use a structured infrared (IR) light source with a known pattern that is projected onto the environment. The distance of individual pixels is then determined from the distortion of the known pattern. As sunlight usually interferes with this sensing mechanism, such RGB-D cameras using projected light are mainly used in indoor environments. Fortunately, newer generations of RGB-D sensors introduce an IR texture projector or time-of-flight (TOF), which are less afected by interferences, supporting outdoor applications.

## 5.1.2 Conversion to Point Cloud

Using the intrinsics of a range sensor $( e . g .$ , a LiDAR or RGB-D camera), we can convert a range image R or a depth map $I _ { D }$ into a point cloud $\mathsf { P } = \{ p _ { 1 } , \hdots , p _ { N } \}$ 2 where points $\pmb { p } _ { i } \in \mathbb { R } ^ { 3 }$ are expressed in the local coordinate frame of the sensor. The point is the most fundamental unit in the map representation and will be discussed further in this chapter.

For conversion from LiDAR, the sensors provide an intrinsic calibration for each beam $( \phi _ { i , j } , \theta _ { i , j } )$ , where $1 \leq$ $i < B$ and $1 \leq j < M$ , consisting of the azimuthal angle $\phi _ { i , j } \in [ 0 , 2 \pi ]$ and polar/inclination angle $\theta _ { i , j } \in [ - \pi , \pi ]$ as depicted in Figure 5.2. Using these known angles of each beam, we can convert a range measurement ${ r } _ { i , j }$ at $R _ { i , j }$ into a three-dimensional point $\pmb { p } = ( x , y , z )$ as follows:

Figure 5.2 Spherical coordinate

$$
x = r _ {i, j} \cos (\theta_ {i, j}) \cos (\phi_ {i, j})\tag{5.2}
$$

![](images/4cb8d131afc13808e90aab558038aa1c2b1707e944026628de3cf58d141cbe72.jpg)

$$
y = r _ {i, j} \cos (\theta_ {i, j}) \sin (\phi_ {i, j})\tag{5.3}
$$

$$
z = r _ {i, j} \sin (\theta_ {i, j}).\tag{5.4}
$$

For an RGB-D camera, we commonly use a pinhole camera model to convert the ranges $r _ { u , v } = R _ { u , v }$ at pixel location $( u , v )$ into a threedimensional coordinate. For this, we use the intrinsics of the camera $\pmb { K } \in \Re ^ { 3 \times 4 }$ to convert a homogeneous coordinate $\pmb { x } = ( u , v , r _ { u , v } )$ in the image coordinate into a point p in the camera coordinate while † being pseudo inverse:

$$
\pmb {p} = \pmb {K} ^ {\dagger} \pmb {x}.\tag{5.5}
$$

The resulting point cloud is said to be organized or unorganized depending on how the points are structured. When converted from a depth map, the point cloud is organized, and each point location is structured with respect to the pixel location of the associate depth map. This can be exploited to compute neighboring points by a simple indexing. On the other hand, the point cloud generated by a LiDAR sensor is more complicated. For example, the generated point cloud is organized in the static 3D mechanical LiDAR. However, the organization of the point cloud no longer holds in the case of non-repeated pattern solid-state, flash LiDARs, or mechanically rotating LiDAR under motion distortion. In many cases, the unorganized points are distorted due to the movement of the sensor and measurement neighborhood in the range image is not necessarily correlated to spatial neighborhood. Therefore, an estimate of the motion of the sensor while completing a sweep, e.g., using inertial measurement unit (IMU) measurements or odometry information, together with the information of the per-beam time is necessary to undistort a LiDAR point cloud to account for the motion of the sensor [1133, 256, 1231]. For more details on motion distortion and compensation, refer to Chapter 8.

![](images/f9bdfadb8891b58ab43430913880377c4f3a68a5a0aec8024ec3acf694c2cc0c.jpg)  
Figure 5.3 Examples of common dense representations.

## 5.2 Foundations of Dense Mapping

A map, generated with sensors’ information and data processing approaches, is a symbolic structure that models the environment [1091, 136]. One thing to be noted here is that the map representation can be diverse, and many diferent representations exist for the same spatial information (as in Figure 5.3). The choice and accuracy of the scene representation strongly impact the performance of the task at hand, and thus the representation should be determined by the use case. For instance, motion estimation and localization in robotics favor sparse representation, such as 3D points features [787, 896] in order to exploit these features for consistent robot pose estimation. On the other hand, a key objective of scene reconstruction is an accurate, dense, and high-resolution map, for example, for inspection purposes [506, 822, 922]. Similarly, path planning tasks require dense information such as obstacle occupancy or closest distance to collision for obstacles avoidance [774, 304, 1110]. Overall, this chapter examines the following three questions, and is organized into sections that address each in turn.

Q1. What quantity do we need to estimate for dense mapping? The most commonly used quantities to represent the environment are occupancy and distance. Occupancy is a key property in mapping for distinguishing between free and occupied space. Distance estimation provides a more robot-centric interpretation of free and occupied space by measuring the range to nearby surfaces or objects.

5.2 Foundations of Dense Mapping  
![](images/4462b836d740234d9efb94fe9df2eec98699164de4853c7ce956d811fcc039f2.jpg)  
Figure 5.4 The representation can be either explicit or implicit. The illustration is simpli fied for clarity. In reality, the abstraction is not clearly separable and can often be applied in a combined manner.

Q2. How should the world be represented? This is the question of what space abstraction we use for representation. Broadly speaking, a representation can be either explicit or implicit. Explicit space abstractions are further classified based on the type of geometry they utilize, while implicit representation can be categorized based on their choice of functions. The list of abstraction types are illustrated in Figure 5.4.

Q3. What data structure and storage should be used? The chosen representation should be stored in memory for later use. The data structure and storage method should be selected based on the specific application and intended usage.

In the literature, a wide variety of approaches exist for generating a dense representation of the scene using range sensors, varying in terms of their estimated quantities, space abstraction, storage structure, continuity, and application areas.

Beginning the discussion on diferent representations, we first explore the primary quantities estimated from range measurements. The focus is on understanding the key quantities predominantly estimated in the mapping phase. We will provide a concise overview of the basic definitions of each quantity, elaborating their significance and the specific contexts in which they play a crucial role.

## 5.2.1 Occupancy Maps

Since their introduction over three decades ago by Elfes and Moravec [304, 774], occupancy grids have been widely used. Their simplicity and computational eficiency have made occupancy grids<sup>2</sup> popular when mapping indoor (and even outdoor) environments. In the simplest scenario, the estimated quantity is the probability of a cell being occupied. In this case, the occupancy of the cell is modeled as a probability of that cell containing an obstacle, with occupancy equal to 1 for occupied cells and 0 for cells deemed empty. Essentially occupancy mapping is a binary classification problem to predict the binary class probability of each cell.

![](images/a0c3917803db17ea2e6794fb0653ac66b0c83186ab12ac2eb9393a3bd34af8dc.jpg)  
Figure 5.5 A simple ray-casting example in 2D. Given a range measurement at a certain (computed or estimated) azimuth, the return of the range measurement indicates the existence of an obstacle.

Given a set of sensor measurements $z _ { 1 : t }$ and a set of sensor poses $\pmb { x } _ { 1 : t }$ , the probability of being occupied for each cell in the map m is modeled as $p ( \pmb { m } | \boldsymbol { z } _ { 1 : t } , \pmb { x } _ { 1 : t } )$ . A sample map is shown in Figure 5.5. Assuming that each cell $m _ { k }$ is independent and that the measurements are conditionally independent, the update of occupancy can be formulated eficiently using the well-known log-odds form [774],

$$
l (m _ {k} | \pmb {z} _ {1: t}) = l (m _ {k} | \pmb {z} _ {1: t - 1}) + l (m _ {k} | z _ {t}),\tag{5.6}
$$

where $l ( \cdot | \cdot ) = \log ( o ( \cdot | \cdot ) )$ , and $o ( \cdot | \cdot )$ is the odds form:

$$
o (m _ {k} | \boldsymbol {z} _ {1: t}) = \frac {p (m _ {k} | \boldsymbol {z} _ {1 : t})}{1 - p (m _ {k} | \boldsymbol {z} _ {1 : t})}.\tag{5.7}
$$

The main advantage of occupancy mapping is shown in (5.6), where only the previous occupancy value and the inverse sensor model $l ( m _ { k } | z _ { t } )$ are needed to update the probability through a simple addition. Despite these benefits, occupancy grids rely on very strong assumptions of the environment to be eficient. Notably, the assumption that the likelihood of occupancy in one cell is independent of other cells disregards spatial correlations, which can be important to infer occupancy in unobserved nearby regions. Additionally, traditional occupancy grids require the discretization of the environment be defined a priori, which makes the spatial resolution constant throughout the map.

## 5.2.2 Implicit Surface and Distance Fields

Another way of representing the geometry of the robot’s surrounding is not through probabilities of occupancy, but rather by describing the boundary between free and occupied space. In an ideal world, we could describe the shape and location of this boundary using an analytical function in three variables (x, y and z) which reaches 0 whenever a point $\pmb { p } = ( x , y , z )$ lies on the surface. In other words, the function’s zero crossings correspond to the surface itself. Such a function is known as an implicit surface. By convention, the function’s sign is negative when p lies inside an object and positive outside. A simple example in Figure 5.6 illustrates how this implicit function values are determined.

![](images/2dc1fb5d9d56f86eadd81aeb50c13cde2fb71fc3f6bf6d80e48c7596ee06da56.jpg)  
Figure 5.6 A solid 2D disk represented as an explicit surface (black outline) and implicit surface (colored field). The implicit surface function is negative inside the object (blue) and positive outside (red). Note how the function’s zero crossings correspond to the surface itself.

There are many advantages to continuous implicit surface representations. Given that there is no fixed resolution, they can represent objects of arbitrary shapes at any level of detail. Furthermore, they make it possible to check if a given point in space is inside or outside an obstacle by simply evaluating the function’s sign.

Diferent types of functions can be employed to model implicit surfaces, with the Euclidean Signed Distance Function (ESDF) being a prevalent option. At any query point, the ESDF expresses the distance to the nearest surface (indicated by the magnitude of the ESDF) and whether the point is inside an obstacle (indicated by the sign of the ESDF). ESDF representations are commonly used by accelerated geometric algorithms for tasks such as collision checking. Furthermore, high-quality proximity gradients can be derived from the ESDF for optimization-based motion planning and shape registration.

The ESDF is computed by finding the (Euclidean) closest surface point for each point in the map. For a known surface, this can eficiently be done using techniques such as Fast Marching. However, for surface estimation, the projective signed distance is more commonly used as it can eficiently be computed from measurements and is better suited for filtering. Given a measurement ray going through a query point, the projective distance is defined as the distance from the beam’s endpoint to the query point along the ray. This eliminates the need to search the closest point explicitly. Although the projective distance overestimates the Euclidean distance, its zero crossings (the estimated surface) remain correct. The standard approach of estimating implicit surfaces, proposed by Curless and Levoy [234] and popularized in the field of robotics by KinectFusion [801], combines the projective signed distances for all measurements using a simple weighted average. To reduce the impact of overestimates, the projective signed distance function is typically clamped to a fixed range, named the truncation band, in which case it is called the Truncated Signed Distance Function (TSDF). Note that ESDFs can eficiently be computed from TSDFs and occupancy maps, as will be described in Section 5.4.4.

## 5.2.3 Occupancy Maps or Distance Fields?

Being volumetric methods, a shared aspect of these estimated quantities in occupancy and implicit representations is that they model the geometry by estimating a quantity of interest everywhere in the observed volume. However, each representation fundamentally prioritizes diferent things. Which option is best, therefore, depends on the application. We will briefly summarize two key diferences.

Directness in modeling: Given that measurement rays directly tell us which parts of space are free, occupied, or unobserved, maps based on occupancy probabilities can be updated using fewer heuristics and assumptions. In contrast, implicit surfaces typically model the distance to the surface. This can be computed exactly for a known surface, but not from partial measurements. For surface estimation, they therefore rely on distance proxies such as the previously introduced TSDF.

Smoothness: Implicit surface maps are inherently smoother than occupancy maps, which model a binary property. The smoothness of implicit surfaces has many benefits. Most importantly, it makes them diferentiable. The resulting proximity gradients are valuable for many applications. Smoothness also reduces the approximation errors resulting from discretization and makes it possible to obtain good, sub-pixel resolution estimates through interpolation. However, since discontinuities cannot be represented smoothly, implicit surfaces tend to miss thin obstacles.

## 5.3 Map Representations

## 5.3.1 Explicitness of Target Spatial Structures

As summarized in Figure 5.4, the representation can be classified based on their explicitness and target space. In 3D mapping, representing volume is straightforward; however, surfaces are equally important in robotic mapping for enabling downstream tasks. We can consider four major categorization: explicit surface, implicit surface, explicit volume, and implicit volume representations.

For surfaces, we can either explicitly or implicitly represent a surface. Defined as a 2D manifold, an explicit type of representation aims to characterize the space in terms of their boundary of the objects in the scene. The simplest abstraction that can represent the boundary is directly the point cloud produced by the range sensors. Another general representation of the surface is the polygon mesh (Section 5.4.3), which comprises vertices, edges, and faces. Meshes have the ability to encode the directed surfaces of a volume by forming connected closed polygons, more commonly triangles. Surfels (Section 5.4.2) are also popular abstractions and are widely used in mapping. Surface representations are a key for any visualization application, but also are used for rendering simulated environments, augmented reality or for Computer-aided design and 3D printing.

Similar strategies are employed in 3D volume modeling. Naive point-based representations are commonly used in LiDAR SLAM. Additionally, occupancy or distance-based voxels (Section 5.4.4) are popular choices for explicit representation. When storing volumetric maps, careful consideration of data storage is necessary to minimize computational costs. Implicit representations for volumetric mapping are also utilized, typically through functions. Gaussian process (GP) (Section 5.4.5) and Hilbert maps (Section 5.4.6) are well-known examples of implicit representations.

## 5.3.2 Types of Spatial Abstractions

## 5.3.2.1 Points

Given range measurements, a straightforward dense map representation is using point clouds. For generation, we accumulate the point clouds $\mathsf { P } _ { t }$ recorded at time t in the local coordinate frame $\mathcal { F } ^ { t }$ using the estimated global pose $\pmb { T } _ { t } ^ { 1 }$ in a global map point cloud $\mathsf { P } _ { M }$ . Also common practice is to assume our global coordinate frame of $\mathsf { P } _ { M }$ is given in the coordinate frame of the first point cloud ${ \mathcal { F } } ^ { 1 }$

Since simply accumulating point clouds $\mathsf { P } _ { 1 } , \ldots , \mathsf { P } _ { t }$ does not scale to larger environments, a common strategy is to discard redundant measurements of the same spatial location. To this end, most methods [1269, 256, 1133] use eficient nearest neighbor search, such as voxel grids or hierarchical tree-based representations (see Section 5.3.3), to subsample and store the point clouds, $e . g .$ , store only a limited number of points per voxel [256, 1133] or only specific points that meet a certain criterion are stored [1269]. Additionally, a representation of only keyframes where only a few point clouds are explicitly stored is possible, but this requires to determine when a keyframe or submap needs to be generated.

Being the most elemental representation form, a point cloud map can be converted into other representations, e.g., a mesh via Poisson surface reconstruction [1131] or a Signed Distance Function (SDF) via marching cubes [1132]. Unfortunately, this is feasible only with additional data, such as the viewpoint of a point’s measurement. Yet, this information may be lost when merging multiple measurements into a point cloud map. Therefore, assumptions about the surface’s direction are often required to discern inside or outside regions.

![](images/729c87cd13c54d9aa29a946444ace672caafb8ea761d9117908e37c7b16390c7.jpg)

![](images/e06e97e692e44fa9c671f1e277e96c47c65f233bb1c254cc891cdcae60f35651.jpg)

(a)  
![](images/e447a927343799a60338d24906b1fd2ccff69f11421e6172382b7553e7a9cdb4.jpg)  
(b)  
Figure 5.7 Qualitative comparison of maps generated by accumulating point clouds and surfels from a sequence of LiDAR scans from the KITTI dataset [373] Sequence 07. (a) Point cloud map. The brightness of points indicates the remission of the LiDAR measurements. (b) Corresponding surfel map based on circular disks. The complete map with all accumulated point clouds use 2.95 GB, while the corresponding surfel map by SuMa [66] uses only 160 MB.

## 5.3.2.2 Surfels

While point cloud maps directly represent the measurements, the stored points do not contain surface information or can represent from which direction a point has been measured. With surfels [870], we can encode such information by adding directional information to a point. Surfels are commonly represented via circular or elliptic discs [549, 66, 1184, 104, 103], or more generally ellipsoids [1041, 1042, 282, 1318] modeled with a Gaussian. Splatting [1318, 103] allows integrating texture information but also blending overlapping surfels into coherent renderings of a specific viewpoint; see Chapter 14 for a more extensive discussion.

A commonly employed circular surfel representing a circular disk is defined by a location $\pmb { p } \in \mathbb { R } ^ { 3 }$ , a normal direction $\mathbf { \boldsymbol { n } } \in \mathbb { R } ^ { 3 }$ , and a radius $r \in \mathbb { R }$ . These geometric primitives can be eficiently rendered using the capabilities of modern graphics processing units (GPUs). This accelerates point-to-surfel associations and leads to a substantial memory reduction.

Figure 5.7 qualitatively compares a point cloud-based and a surfel-based map representation. A dense point cloud can accurately represent the environment with a high level of detail, but at the cost of memory. In contrast, while losing fine details as multiple measurements get aggregated into a single surfel, significant memory usage can be reduced while preserving the main structural details of larger surfaces.

Closely related to the explicit geometric representation of surfaces via surfels, $i . e . ,$ , small circular surface patches, is the representation via a normal distributions transform (NDT) [82, 1035]. Using NDT, the space is subdivided into voxels and the points inside a voxel are approximated via a normal distribution $\scriptstyle { \mathcal { N } } ( \mu , \Sigma )$ , having estimated mean $\pmb { \mu }$ and covariance from the enclosed points Σ. The eigenvalues $\lambda _ { 1 } < \lambda _ { 2 } < \lambda _ { 3 }$ and corresponding eigenvectors ${ \pmb v } _ { 1 } , { \pmb v } _ { 2 } , { \pmb v } _ { 3 }$ of the covariance can be used to estimate the surface properties inside a voxel. For planar surfaces $\left( \lambda _ { 1 } \ll \lambda _ { 2 } \right)$ ， the eigenvector ${ \pmb v } _ { 1 }$ of the smallest eigenvalue $\lambda _ { 1 }$ corresponds to the surface normal. Thus, for planar surfaces the NDT represents a surfel, and can also more accurately represent point distributions that cannot be approximated via a surfel. In that sense, the NDT is a hybrid representation that is explicit due to the space division into a voxel grid, but also implicit due to the representation of voxels via a normal distribution which continuously represents the space inside a voxel.

## 5.3.2.3 Meshes

While describing local surface properties, both point clouds and surfel maps are still relatively sparse as they do not model the surface’s connectivity. One way to get a more complete understanding is to use meshes, which describe the surface as a set of points that are connected to form a collection of polygons. This, in turn, makes it possible to represent watertight surfaces, query and interpolate new surface points, and eficiently iterate along a connected surface.

In meshing terminology, each polygon is referred to as a face, and each corner point as a vertex. The most common types are triangle meshes, where each face is bounded by three vertices, and quad meshes, whose faces are bound by four vertices. Note that a polygon, or face, can always be broken down into an equivalent set of triangles; hence, triangle meshes are not only the simplest but also the most general.

A mesh is a very flexible and memory-eficient representation because the number of faces and vertices can be directly adapted to the surface complexity and required level of detail. For example, a plane of any size can be represented with just two triangular faces and four vertices. Furthermore, meshes are well-suited for parallel processing and rendering. Meshes are often used in applications that overlap with computer graphics, such as rendering, surface analysis, manipulation, and deformation, and more generally in applications involving digital models, simulation, or surface-based algorithms, such as path planning for ground robots.

## 5.3.2.4 Voxels

Point clouds and meshes are well suited to represent properties of the environment that are defined along surfaces. However, certain estimated quantities, including occupancy and Signed Distance, are defined throughout the entire volume. One straightforward way to store and process volumetric properties is to discretize them over a regular grid. Discretized occupancy and Signed Distance maps are called occupancy grids and Signed Distance Fields, respectively.

Generalizing the concept of 2D pixels, the cells in a 3D grid are referred to as voxels. Given a grid’s regular structure, each voxel can easily be assigned a unique index and stored in a data structure. Note that a voxel is merely a container or, more formally, a space partition. The significance of its contents varies from one method to another. In a classic occupancy grid, a voxel’s value represents the likelihood that any point in the voxel is occupied. Hence, the occupancy at an arbitrary point in the map is equal to the value of the voxel that contains the point. However, a voxel’s value does not have to represent a constant little cube. For example, voxels in a Signed Distance Field estimate the signed distance at each voxel’s center. To retrieve the signed distance at an arbitrary point, one would therefore query the voxels that neighbor the point and obtain the point’s value using interpolation. Finally, some applications even use (sparse) voxel grids to store and eficiently query non-volumetric properties, such as points or surface colors.

## 5.3.2.5 Continuous Functions

Functions are a key abstraction for mapping in a continuous manner. The problem of mapping in this case is reduced to fitting a parametric or non-parametric function, i.e., solving a regression problem. Most of the above-mentioned space abstractions require the discretization of the environment to be defined a priori, which usually makes the spatial resolution constant throughout the map. Continuous functions, however, parametric or non-parametric, give more flexibility allowing the resolution to be recomputed and also provide interpolation capabilities to fill up data gaps.

Some parametric functions such as infinite lines in 2D [1106] and planes in 3D [533, 374, 1106] require making strict assumptions about the environment and limit the representation of the scene. However, these representations are eficient in terms of memory consumption and computational complexity. Control pointsbased functions (e.g., B-splines [932]) or non-parametric (e.g., GP-based) have the ability to model the environment with fewer assumptions, still in a continuous manner. From occupancy [816], implicit surface [1186, 636], distance fields [1195], and surface itself [1122], GP-based representations are a popular choice to represent the environment —despite their high computational complexity— because of their probabilistic nature, which enables uncertainty quantification and inference over both observed and unseen areas [378].

A key advantage of the continuous functions for mapping is that if they are chosen to be at least once diferentiable they will be able to provide gradients. Gradient information can be key for localisation to compute surface normals [1196], loop closure to compute terrain features [377], for data fusion [636] and planning [1196] applications.

![](images/bbd51a8abb86d14f4b9dbf9318ccd6d5f8876ce59b2e1abd8d16e42de8529bd1.jpg)  
Figure 5.8 The Marching Cubes algorithm works by projecting the cubes into the implicit surface, querying the sign of the values at the corners of the cubes, and looking up which one of the 15 configurations these values map to. (Image credit: Ryoshoru, “Marching cubes”, licensed under CC BY 4.0. Source: https://commons.wikimedia.org/wiki/File:MarchingCubesEdit.svg)

## 5.3.2.6 Conversions

The abstractions listed above are not always used exclusively; they are often converted from one form to another or employed simultaneously in multiple forms.

For instance, explicit geometry, such as points and meshes, can be transformed into an implicit surface. One flexible method to compute the signed distance at any point in space is through a closest point lookup, which can be performed against any explicit geometry and on-demand, only when and where needed. Alternatively, the signed distance can be computed across all points on a regular grid using wavefront propagation, which can eficiently be implemented via the fast marching method [991].

Conversely, it is common to convert implicit surfaces into mesh representations. The original technique for converting distance fields into meshes is known as Marching Cubes [696]. The algorithm divides the implicit surface into a grid of fixed-size cubes, which it processes independently. Each cube generates a set of triangle elements based on the implicit surface’s values at its eight corners (Figure 5.8). The positions of their vertices are then refined through linear interpolation. Meshes, including those from BIM (Building Information Modeling) and CAD (Computer-Aided Design) models, can, in turn, be sampled to create points or surfels.

Occasionally, a discrete representation must be converted into a continuous one.

![](images/8ee90dccf0e08060ba9abfa39ab992f214f72150d351a5c34c0a1ab203b79f72.jpg)  
Figure 5.9 (a) Mapping of logical grid coordinates to an underlying naive array-based storage through a function f(·) that uniquely maps coordinates to linear indices. (b) Storage of unordered data directly into an array structure.

This is typically achieved by solving an optimization problem over the parameters of the continuous representation, minimizing the fitting error with respect to the discrete data.

## 5.3.3 Data Structures and Storage

The abstractions introduced in the previous sections all need to be stored in memory. In this section, we explore how various abstractions are stored in memory by examining the choice of data structures along with their advantages and disadvantages.

## 5.3.3.1 Naive Data Storage

For many representations a simple dynamically resizeable array is a reasonable starting point. For data with a pre-defined spatial partitioning two things are needed, the type of data to store and a conversion function from a spatial coordinate to an index coordinate. This is often used when building maps representing occupancy or signed distance values. For irregular data, only the type of data to store is needed, for example point clouds or surfels. The naive storage of ordered data using a mapping function and unordered pointcloud data is illustrated in Figure 5.9.

The benefit of this naive approach is that it is simple and provides fast random access. The trade-of is, that large amounts of memory can be required for such a representation. Also while read and modify operations are fast, changing the spatial dimension of the representation can be very costly as the content of the entire data structure needs to be copied.

## 5.3.3.2 Hash Map

A natural extension to address the limitations of the naive storage method described above is to use a hash table. This approach divides the map into shards, applies a hash function to convert the coordinates of each shard into a single value, and stores the sharded data in a table indexed by this hash value. The shards are typically chosen to represent map subregions with well-established coordinates, such as cubes in a regular grid. These cubes may correspond to individual voxels or fixedsize groups of voxels, referred to as voxel blocks. Alternatively, they can also store other elements like points, surfels, or mesh fragments contained in their respective subregions.

A hash table retains the fast (1) look-up time of a fixed array while allowing the map to grow dynamically without reallocation. Three key considerations must be addressed when using a hash table for dense map storage:

1 Granularity of the sharding: Smaller shards improve sparsity by allocating data only where necessary. However, the number of shards should not grow too large, as this reduces the hash table’s insertion performance and memory eficiency. This trade-of is particularly relevant when hash maps are used to store properties that only exist along the surface.

2 Hash function: An ideal hash function distributes keys evenly across the table, even when the data is spatially adjacent, as is often the case in mapping scenarios.

3 Collision resolution: The method for handling hash collisions, whether through linear chaining (where each entry contains a linked list) or open addressing, significantly afects the performance of the hash table.

In most cases, hash tables ofer a good balance of fast access and eficient insertion and deletion of data. However, they may require initial tuning to perform well for a given application.

## 5.3.3.3 Tree-based Data Structures

Another option to eficiently store spatial data while only occupying memory for relevant parts of the environment is to use hierarchical, tree-based representations. Just like hash tables, trees generally enable eficient access and insertions. However, their unique strength is their hierarchical structure, which can be used to eficiently store multi-resolution data and speed up spatial operations such as nearest neighbor search. The most prominent tree variants are kD-trees [72], bounding volume hierarchies (BVH) [223], and octrees [752].

Among them, the octree eficiently searches neighbors with the capability to integrate novel measurements incrementally. The octree is a tree representing each node by a so-called octant that refers to a subspace. An octant is defined by a center $\boldsymbol { c } \in \mathbb { R } ^ { 3 }$ and an extent $e \in \mathbb { R }$ , corresponding to an axis-aligned bounding-box. Each octant has potentially 8 child octants of extent ${ \begin{array} { l } { { \frac { 1 } { 2 } } e , } \end{array} }$ as depicted in Figure 5.10.

![](images/62d2224194886c41e6b38d4a72042e4c1051e1b13121ba09342f5ad28a959c2f.jpg)  
Figure 5.10 Example of an octree and its octants at diferent levels of the tree hierarchy. Each level of an octree subdivides the space in more fine-grained octants. Note that deeper levels of the octree only represent the occupied space.

Common practice is to store points only in the leaf octants (i.e., octants without children) and determine subsets of points at inner octants of the tree structure by tree traversal.

To construct an octree, we iteratively divide space into octants within an axisaligned bounding box encompassing a point cloud P. Each division splits P into subsets $\mathsf { P } _ { 1 } , \ldots , \mathsf { P } _ { 8 } ,$ , corresponding to 8 child octants of half extent ${ \begin{array} { l } { { \frac { 1 } { 2 } } e . } \end{array} }$ Non-empty subsets $\mathsf { P } _ { i }$ form child octants with center c and extent $\textstyle { \frac { 1 } { 2 } } e$ , stopping at a specific octant size or a minimal point count. Once constructed, updates and insertions can eficiently be performed by traversing the tree structure and adding inner nodes as needed. When new data is inserted that falls outside of the tree’s root octant, the tree can be extended by creating a new root node and assigning the new data and the old root node to its children.

In contrast to voxel grids, an octree represents only data-containing subspaces, enabling eficient storage of occupied space. However, this memory eficiency requires tree traversal to access specific leaf octants, potentially leading to increased runtime to locate points. Additionally, the tree structure itself must be explicitly represented, incurring extra memory overhead. Several recent approaches address these memory overheads [305, 820, 70].

## 5.3.3.4 Hybrid Data Structures

To balance memory requirements and runtime for data access, several data structures combine the advantages of diferent data structures in specific ways, leading to hybrid representations. In this tradeof, we accept less eficient memory usage but enable more eficient memory access.

For example, hashed voxel grids [808] combine the strengths of dense voxel grids and hash tables by splitting the environment into fixed-sized, dense blocks (e.g. 8 8 8 voxels), which are in turn stored in a hash table. Thanks to the hash table’s flexibility, blocks only need to be allocated in locations that contain meaningful information (e.g. near the surface). At the same time, using a plain 3D array to store the voxels inside each block ensures that operations remain simple, eficient, and even suitable to GPU acceleration.

Another option is to combine hash tables with trees. In a similar vein to hashed voxel grids, the VDB <sup>3</sup> data structure [792, 791] splits the space into hashed blocks, but stores a hierarchical tree inside each block. This data structure provides all the benefits of hierarchical trees, including multi-resolution representation and eficient nearest neighbor lookups. However, since each block has a fixed size, the maximum tree height is constant regardless of the size of the environment. Lookups and insertions can therefore be performed in constant time, and significantly faster than when using pure trees.

5.4 Constructing Maps: Methods and Practices

<table><tr><td>Section</td><td>Space Abstraction Type</td><td>Representing Map Entities</td></tr><tr><td>5.4.1</td><td>Points</td><td>Surface</td></tr><tr><td>5.4.2</td><td>Surfels</td><td>Surface</td></tr><tr><td>5.4.3</td><td>Mesh</td><td>Surface (connected)</td></tr><tr><td>5.4.4</td><td>Voxels</td><td>Occupancy or Implicit surface</td></tr><tr><td>5.4.5 - 5.4.6</td><td>Continuous function</td><td>Occupancy or Implicit surface</td></tr></table>

Table 5.1 Summary of presented mapping methods.

## 5.4 Constructing Maps: Methods and Practices

So far, we have explored the target quantities to estimate and the various space abstractions available for mapping. In this section, we will examine in detail the methods used to construct these map elements. The approaches are categorized by their main space abstraction, as shown in Table 5.1. Note that some of the methods use additional space abstractions to improve performance, for example, by grouping points into voxels for more eficient storage and faster queries.

## 5.4.1 Points

As mentioned in Section 5.3.2.1, naively storing points by accumulating the measured point clouds will not scale to large-scale environments and will lead to redundantly represented measurements. Therefore, most approaches [1269, 256, 1133] adopt a point-based representation in combination with a voxel grid or octree to represent the dense map. Moreover, the selection of a data structure is driven by the requirement for eficient nearest neighbor searches, essential for conducting scan registration through iterative closest point (ICP), where point correspondences must be iteratively established.

In order to handle large-scale environments, some methods, such as the wellknown LiDAR SLAM LOAM [1269], filter the raw point clouds to extract corner and surface points thereby significantly reducing the point cloud size. A voxel grid is applied to store only a subset of points in the map representation, pruning redundant measurements. Stemming from the point-based voxelization used in LOAM, several follow-up approaches [998, 1150, 839, 667, 894, 999] refine the extraction of points [998, 1150, 839], improve the optimization pipeline [839, 667], or integrate information of an IMU [894, 999].

Another branch of methods handles the amount of point cloud data diferently to avoid reliance on a capable feature extraction approach. Regularly sampling the point clouds via a voxel grid [256] significantly reduces the number of points per LiDAR scan and removes potentially redundant information. The key insight is here that points in the voxel grid are not aggregated and averaged, but original measurements are retained. Following these insights, Dellenbach et al. [256] and Vizzo et al. [1133] use this strategy to downsample an input point cloud, only storing a restricted number of points inside a voxel grid map.

Overall, as also mentioned in Section 5.3.2.1, the (hashed) voxel grid serves dual purposes: it abstracts space by storing a limited number of point measurements per voxel, and it facilitates accelerated nearest neighbor search through direct indexing of neighboring voxels.

## 5.4.2 Surfels

For surfels, similar strategies can be applied as for point clouds, but notably St¨uckler and Behnke [1041] and follow-up work by Droeschel and Behnke [282] use an octree to represent surfels at multiple levels in the octree hierarchy for data association. The so-called multi-resolution surfel maps indirectly represent the surfels via accumulated mean and covariance statistics, like a NDT.

In contrast, Whelan et al. [1184] store surfels as a simple list and exploit eficient rendering techniques to produce a projection for data association for RGB-D SLAM in indoor environments. In this case, surfels are explicit geometric primitives and, therefore, need to be directly handled to update the surfel properties (i.e., size and direction) accordingly [549]. A key contribution of Whelan et al. [1184] is leveraging a map deformation that directly deforms the surfels instead of relying on a PGO, which enables the use of the measurements represented by the surfels to deform the map on a loop closure detection. A similar strategy for map deformation of surfels was employed by Park et al. [846].

Similarly, Behley and Stachniss [66] target outdoor environments, which makes it necessary to represent the surfels via multiple submaps of 100 m 100 m spatial extent that can be of-loaded from GPU memory. In contrast to ElasticFusiuon [1184], the approach relies on PGO but exploits that surfels can be freely positioned and ties surfels to poses enabling a straight-forward deformation of the map with posegraph-optimized poses, which was also adopted by other approaches [1158].

## 5.4.3 Meshes

As introduced ealier, meshes ofer an expressive, flexible way to represent connected surfaces. Mesh generation methods can be split into two families of approaches. The first family directly converts the measured points into a mesh. In contrast, the second family splits the problem into two steps: reconstructing an implicit surface, followed by iso-surface extraction to get the final mesh (see Section 5.3.2.6).

Methods in the first family typically work directly by computing the Delaunay triangulation of the input point set and identifying the subset of Delaunay triangles that lie on the surface. A detailed overview of such methods is provided in [157]. When building directly from points, the mesh implicitly adapts itself to the sampling density. This can be an advantage, as it provides adaptive resolution, but it also means these methods are more sensitive to sampling irregularities and holes. In practice, direct meshing methods are chosen when the entire surface can be sampled densely with a very accurate depth sensor, for example, using surveying equipment.

The second family of approaches uses an implicit surface as an intermediate step, to simplify the process of fusing and filtering the data before extracting the final surface mesh. One intuitive way to generate the implicit surface from data is to estimate the distance to the surface at each point on a regular grid. As described in Section 5.2.2, the implicit surface’s sign must also be set according to whether each point is inside or outside an object. This information is often determined based on estimated surface normals, which can for example be obtained by applying Principal Component Analysis (PCA) over a small surrounding area. However, as indicated in [476], such methods may yield implicit surfaces that are discontinuous. Tackling this issue, Carr et al. [155] model the implicit surface using a collection of Radial Basis Functions (RBFs) and fit these to the input points by solving a global optimization problem. The resulting implicit surfaces are smooth by construction and faithfully fill holes based on the global context. Unfortunately, solving the underlying large, dense optimization problem is computationally expensive. Shen et al. [1002] overcome this limitation by locally approximating the input points using moving least squares (MLS). Going one step further, Poisson Surface Reconstruction [545] fits the implicit function to the normals of the measured points by solving a partial diferential equation (PDE), resulting in a sparse, computationally tractable optimization problem that is particularly robust to noise.

In robotics applications, constructing a mesh from a live sensor stream is often desirable. One way to make surface reconstruction eficient enough to run in real-time is to use incremental updates. TSDF-based surface reconstruction is particularly popular in practice given its inherently incremental nature and general simplicity. This method falls under the second family of approaches and estimates the implicit surface by averaging projective distances. Since the cost of updating the TSDF, or implicit surface in general, overshadows the cost of the mesh extraction, real-time methods primarily focus on optimizing the former.

## 5.4.4 Voxels

Voxel-based methods are among the most commonly used volumetric representations in 3D reconstruction and robotics. Instead of covering a swath of existing literature chronologically, this section will focus on concepts commonly encountered in practice and organize them according to three fundamental decision criteria: the chosen estimated quantity, data structure, and scalability considerations.

## 5.4.4.1 Methods by their Estimated Quantity

The first choice in a voxel-based mapping framework is which quantity to estimate, with the most common options being occupancy (see Section 5.2.1) or a distance metric (see Section 5.2.2). The previous discussion in Section 5.2.3 can be used to decide between the two.

Since the introduction of the original continuous probabilistic occupancy measurement model for sonar [774], simplified piecewise-constant models have been developed to reduce computational costs [478]. This shift was influenced by the advent of LiDAR technology and the growing interest in transitioning from 2D to 3D maps. More recently, Loop et al. [692] presented a continuous probabilistic model that, instead of inflating objects, converges to an occupancy probability of 0.5 along objects’ surfaces. Occupancy estimation, popular for collision avoidance due to its superior recall, is limited by its discontinuous nature and uninformative gradients compared to distance-based methods (see Section 5.2.3).

For distance metrics, we must not only estimate the positive part of the distance field but also extrapolate negative distances behind the surface since the surface is represented by the signed distance field’s zero-crossings. To limit the accuracy impact of fusing imperfect positive and negative distances estimates (see Section 5.2.2), the updates are typically clamped to a small truncation band around the surface boundary. However, distance-based methods remain prone to erasing geometry. For example, when thin objects are observed from opposing sides, averaging the observed positive and hallucinated negative distances makes the zero-crossings flip around or disappear. Some works have analyzed the efect of the truncation band and weight drop-ofs on the quality of the final reconstruction [134]. Fundamentally, the problem can be reduced but not eliminated. Overall, the surfaces estimated by TSDFs outperform occupancy methods along smooth surfaces at the cost of lower recall on thin objects.

The distance information provided by TSDFs is inherently valuable. However, instead of being conservative, TSDFs strictly overestimate the Euclidean distance. To address this safety concern, voxblox [822] popularized incrementally building ESDFs. Voxblox fuses the sensor data into a TSDF and then updates its ESDF using a brushfire algorithm [629]. Subsequently, FIESTA [426] proposed a hybrid approach that incrementally updates an ESDF map from an occupancy map instead.

## 5.4.4.2 Methods by Data Structure

The simplest data structure for volumetric mapping is a static 3D array. As shown by KinectFusion [801], this data structure yields good results for small and fixedsize scenes. However, many applications require the ability to dynamically expand the map at runtime, while only allocating voxels where needed to save memory.

To address these concerns, Nießner et al. [808] proposed a voxel-block hashing scheme, which groups the voxels into blocks (e.g., 8 8 8 voxels) that are stored in a hash-map. This data structure was quickly adopted for TSDFs, providing constanttime ( (1)) lookups and dynamic insertions. Of course, it can also be used to store occupancy probabilities, as shown by FIESTA [426]. Compared to hashing voxels individually, grouping them in blocks ofers an adjustable trade-of between the hash table’s size and the granularity at which voxels are allocated.

Naturally, voxels can also be stored using tree structures. Octomap [478] first popularized using an octree to store occupancy probabilities and has been the de facto standard for volumetric mapping for many years. A significant advantage of using trees is that they inherently support multi-resolution, while a major limitation is that encoding the tree’s structure introduces a significant memory overhead, and that the cell lookup time is proportional to the tree’s height. Most recent approaches address this limitation by leveraging hybrid data structures. Supereight [1125], for example, proposes to use a standard (dynamic) octree for the first levels and static octrees for the last few levels. These static octrees can be seen as octrees stored using a fixed-sized array. This removes the memory overhead of encoding parent-child relationships with pointers, at the cost of reducing granularity since static octrees are allocated as a block. The VDB [792] data structure was first introduced for the visual efects (VFX) industry and subsequently used by several volumetric mapping frameworks [721, 1132]. As discussed in Section 5.3.3.4, it combines block-hashing with trees to obtain the best of both worlds: good memory eficiency, hash-like constant time lookups and insertions, and tree-like multi-resolution.

A practical consideration is that downstream tasks often demand storing additional information, such as colors, semantics [398, 942] or an ESDF [822, 426] alongside the occupancy probabilities or TSDF. Although virtually any data structure can be extended to support additional channels, the required implementation efort scales with how complicated the underlying data structure is. This further motivates using simple data structures (e.g. voxel-block hashing) or flexible, thirdparty libraries.

## 5.4.4.3 Methods by Measurement Integration Algorithm

The algorithm used to update the map based on depth measurements is referred to as the measurement integrator. It updates the estimated quantity for each observed voxel by applying the measurement model. The two main approaches used to integrate measurements are ray-tracing and projection-based methods.

For each measured point, ray tracing integrators cast a ray from the sensor to the point and update all the voxels intersected by the ray. An advantage of this approach is that it is very general, and only requires that the position of the sensor’s origin is known. However, voxels may be hit by multiple rays, especially if they are near the sensor. This leads to duplicated eforts, and handling the resulting race conditions in parallel implementations creates implementation and performance overheads.

In contrast, projection-based methods directly iterate over the observed voxels and look up the ray(s) needed to compute their update by projecting each voxel into sensor coordinates. Iterating over the map instead of the rays inherently avoids race conditions. Projection-based methods are, therefore, prevalent in multi-threaded and GPU-accelerated volumetric mapping frameworks. The predictable access pattern resulting from directly iterating over the map also reduces memory bottlenecks. Yet, a major disadvantage is the need for explicit knowledge of the sensor’s full pose and projection model. This method is also harder to use with disorganized point clouds, including the clouds obtained after applying LiDAR motion-undistortion.

## 5.4.4.4 Methods by Scalability

Memory and computational costs are two of the main bottlenecks in volumetric mapping. For fixed-resolution methods, the memory and computational complexities grow linearly with the map’s total volume and cubically with the chosen resolution. Reducing these complexities is of significant research interest, as it is necessary to create detailed maps that scale beyond small, restricted volumes.

Early works in volumetric mapping mainly focused on reducing memory usage. For example, Octomap [478] proposes to use its octree’s inner nodes to store their children’s max or average occupancy. By recursively pruning out leaf nodes whose estimated quantities are close to their parent, constant areas in the map are automatically represented with fewer, lower resolution nodes. This adaptation to the environment’s geometry is very efective in practice since environments predominantly consist of free space. Furthermore, storing min, max, or average values in the octree’s inner nodes could be valuable for downstream tasks, as it enables map queries at lower resolutions and the use of hierarchical algorithms for tasks such as fast collision checking or exploration planning. Yet, a core limitation of Octomap is that it integrates all measurements at the highest resolution, meaning that the scaling of its computational complexity remains cubic.

Multi-resolution can also be leveraged to reduce the computational cost of measurement updates. Given that measurement rays are emitted at fixed angles, resulting in fewer rays hitting distant geometry, it seems logical to lower the update resolution as the distance increases. This can be achieved through multi-resolution ray-tracing [286] or multi-resolution projective integration [1125]. Supereight2 [347] reduces the computational complexity further by adjusting the update resolution to the entropy of the measurement updates. Such methods significantly enhance the update performance, yet a remaining challenge is that the map’s diferent resolution levels still have to be synchronized explicitly. One way to eliminate this synchronization requirement is to encode only the diferences between each resolution level, instead of storing absolute values in each octree node. This can formally be done by applying wavelet decomposition. Wavelet-encoded maps can eficiently be queried at any resolution at any time. Using this property, wavemap [923] reduces the computational complexity even further by updating the map in a coarse-to-fine manner. In addition to adjusting the update resolution to the measurement entropy, it also skips uninformative updates, such as when the occupancy for an area in the map has converged to being free, and all measurements agree.

## 5.4.5 Gaussian Processes

As mentioned in Section 5.3.2.5, formulating the mapping problem as a regression problem is desired to obtain a continuous representation. Moreover, if the aim is to limit the number of assumptions about the environment, solving a non-linear regression problem with non-parametric methods is ideal. A Gaussian process (GP) [909] is a stochastic, non-parametric, non-linear regression approach. It allows estimating the value of an unknown function at an arbitrary query point given noisy and sparse measurements at other points. We already learned in Chapter 2.2 how GPs can be used for continuous time trajectory representation. As will be apparent, GPs are also an appealing solution for mapping continuous quantities, and they have been extensively used in the robotics literature to model continuously spatial phenomena with depth sensors [1122, 816, 377, 574].

The information in GP models is contained in its mean ${ \pmb m } ( { \pmb x } )$ and kernel functions $\kappa ( \pmb { x } , \pmb { x } ^ { \prime } )$ and model the estimated continuous quantity as

$$
f (\boldsymbol {x}) \sim \mathcal {G P} \left(\boldsymbol {m} (\boldsymbol {x}), \boldsymbol {\mathcal {K}} (\boldsymbol {x}, \boldsymbol {x} ^ {\prime})\right).\tag{5.8}
$$

Let $\mathsf { X } = \{ \pmb { x } _ { j } \in \mathbb { R } ^ { D } \}$ be a set of locations with measurements y, with $y _ { j } =$ $f ( \pmb { x } _ { j } ) + \epsilon _ { j }$ of the estimated quantity taken at the locations $\mathbf { \Delta } _ { \mathbf { \mathcal { X } } _ { j } }$ . For $J$ number of training pair $( \boldsymbol { x } _ { j } , \boldsymbol { y } _ { j } )$ , we assume the noise $\epsilon _ { j }$ to be i.i.d following Gaussian $\epsilon _ { j } \sim \mathcal { N } ( \mathbf { 0 } , \sigma _ { j } ^ { 2 } )$ . Given a set of testing locations $\mathsf { X } ^ { * } = \{ \pmb { x } _ { n } ^ { * } \in \mathbb { R } ^ { D } \mid n = 0 , \ldots , N \}$ we can express the joint distribution of the function values and the observed target values as,

$$
\left[ \begin{array}{c} \boldsymbol {y} \\ \boldsymbol {f} _ {*} \end{array} \right] = \mathcal {N} \left(\mathbf {1}, \left[ \begin{array}{c c} \boldsymbol {K} (\mathrm{X}, \mathrm{X}) + \sigma_ {j} ^ {2} \mathbf {I} & \boldsymbol {K} (\mathrm{X}, \mathrm{X} ^ {*}) \\ \boldsymbol {K} (\mathrm{X} ^ {*}, \mathrm{X}) & \boldsymbol {K} (\mathrm{X} ^ {*}, \mathrm{X} ^ {*}) \end{array} \right]\right),\tag{5.9}
$$

where $\pmb { K } = \left[ \pmb { \mathcal { K } } ( \pmb { x } _ { i } , \pmb { x } _ { j } ) \right] _ { i j }$ . Thus the conditional distribution of $( f _ { * } \mid \mathsf { X } , y , \mathsf { X } ^ { * } ) \sim$ $\mathcal { N } \left( \overline { { \pmb { f } } } _ { \ast } , \mathrm { c o v } \left( \pmb { f } _ { \ast } \right) \right)$ , with the mean equation is given by,

$$
\overline {{\boldsymbol {f} _ {*}}} = \boldsymbol {K} (\mathsf {X} ^ {*}, \mathsf {X}) [ \boldsymbol {K} (\mathsf {X}, \mathsf {X}) + \sigma_ {j} ^ {2} \mathbf {I} ] ^ {- 1} \boldsymbol {y},\tag{5.10}
$$

and the covariance equation is,

$$
\operatorname{cov} \left(\boldsymbol {f} _ {*}\right) = \boldsymbol {K} \left(\mathrm{X} ^ {*}, \mathrm{X} ^ {*}\right) - \boldsymbol {K} \left(\mathrm{X} ^ {*}, \mathrm{X}\right) \left[ \boldsymbol {K} (\mathrm{X}, \mathrm{X}) + \sigma_ {j} ^ {2} \mathbf {I} \right] ^ {- 1} \boldsymbol {K} \left(\mathrm{X}, \mathrm{X} ^ {*}\right).\tag{5.11}
$$

Here, (5.10) and (5.11) are the predictive equations for the estimated quantitative. GPs have proven particularly powerful to represent spatially correlated data, hence overcoming the traditional assumption of independence between cells, characteristic of the occupancy grid method for mapping environments. Gaussian Process Occupancy Map (GPOM)s [816] collects sensor observations and the corresponding labels (free or occupied) as training data; the map cells comprise testing locations, which are related to the training data as shown in (5.9). After the regression is performed using (5.10) and (5.11), the cell’s probability of occupancy is obtained by “squashing” regression outputs into occupancy probabilities using binary classification functions.

In its original formulation GPOM is a batch mapping technique with cubic computational complexity $\left( \mathcal { O } ( J ^ { 3 } + J ^ { 2 } N ) \ \right)$ . Approaches that aim to tackle this computational complexity —especially for incremental GP map building— have been proposed following this work, for example [574, 575, 1157, 378].

A key advantage of mapping with GP-based functions is that the estimated quantity can be linearly operated [969] and still produce a GP as an output. Given that derivatives and, therefore gradients, are linear operations, the diferentiation output of the estimated quantity is probabilistic. A continuous representation of the uncertainty in the environment can be used to highlight unexplored regions and optimize a robot’s search plan [378, 677]. The continuity property of the GP map can improve the flexibility of a planner by inferring directly on collected sensor data without being limited by the resolution of a grid/voxel cell.

## 5.4.5.1 Gaussian Process Implicit Surface

Implicit surfaces can also be represented by a GP. Gaussian process implicit surface (GPIS) techniques [1186, 736, 677, 505] use a GP approach to estimate a probabilistic and continuous representation of the implicit surface given noisy measurements. Furthermore, GPIS can be also used to estimate not only the surface but also the distance field in a continuous manner [576, 1034, 636, 1195].

In the GPIS formulation, let us consider the distance field d to be estimated from the distance to the nearest surface $d _ { i }$ given the points on the surface and its corresponding gradient d computed through linear operators [969]. Then d with d can be modelled by the joint GP with zero mean (given that at the surface the distance is zero):

$$
\left[ \begin{array}{c} \boldsymbol {d} \\ \nabla \boldsymbol {d} \end{array} \right] \sim \mathcal {G P} (\boldsymbol {0}, \boldsymbol {K} (\mathsf {X}, \mathsf {X} ^ {\prime})).\tag{5.12}
$$

GPIS approaches have the ability to estimate a continuous implicit surface and the normal of the surface through the gradient, both with uncertainty. Some works have considered the use of parametric function priors to capture given shapes more accurately [736, 505]. Other approaches aimed to estimate not only the implicit surface but the full distance field. Given the nature of the vanilla version of the GPIS formulation, the distance is well approximated near the measurements, i.e., on the surface, but falls back to the mean, which in this case is zero, faraway from the surface. To estimate the full distance field in a continuous and probabilistic formulation further away from the surface, works have considered applying a nonlinear operation to a GPIS-like formulation [1195, 1196, 634].

All these works have to deal with the computational complexity of the GPbased formulation, but as an exchange, a continuous, generative, and probabilistic representation of the environment, given only point clouds, can be achieved.

## 5.4.6 Hilbert Maps

Hilbert Maps (HMs) [904] are in many ways similar to GPOMs [816]. Both are continuous probabilistic models that do not discretize the space, unlike voxel-based methods, and in contrast to point-based methods are capable of interpolating missing data. As stated, the major challenge in GPOM is high computational expense. Thus the design goals of Hilbert maps were the following: (i) process data continuously in an online manner, (ii) model dependence between observations, and (iii) incorporate measurement uncertainty.

To achieve these goals, training a logistic regressor with stochastic gradient descent in a projected feature space is often leveraged. The classifier and optimizer combination enables online model updates using large amounts of data while the feature projection permits representing intricate spatial details with such a simple classifier.

The feature projection serves the same idea as the kernel in a GP, but instead of a full covariance we use an approximation. There are many options for this, including Nystroem [1185], Random Fourier Features [901], and Sparse Kernel [755], which is what we will be using. To goal of the sparse kernel is to limit the range at which observations have an influence which improves convergence and computational eficiency. The outcome is a kernel that drops to exactly 0 at a specific distance.

This kernel allows us to project points in 2D or 3D space into significantly higher dimensions by placing inducing kernels at regular intervals over the space to be mapped. Furthermore, this enables computing high-dimensional feature space representations of input data to be trained the logistic regression classifier using minibatch stochastic gradient descent. Lastly, training is done by sampling free space points along the range measurement, while adding the return as an obstacle point.

One challenge faced by HMs is the expressivity of the used kernel. A radial basis function (RBF), as used in Figure 5.11, is a circle or a sphere and their values need to be combined to reconstruct intricate details of the environment. Therefore there is a trade-of in the form of number of kernels and their length-scale afecting the computational cost, reconstruction detail, and interpolation ability.

![](images/5747e487164f38c4984a894030b3612ceeef8b8343c44325b8dcab30ec69c1ad.jpg)  
(a)

![](images/c41a8880f986d378ffdd3b2b788bf1040a220b44cc3e9cb23aca49971e288ad2.jpg)  
(b)

![](images/94ad4836cb31f41ddbc193757247100d5789dc5d43b9b9ccf2d591faf82cd7ac.jpg)  
(c)  
Figure 5.11 (a) The observations by a robot using a 2D LiDAR sensor, which is turned into a dataset of (b) Free (green) and occupied (red) points. These are then used to train a Hilbert Map as seen in (c).

## 5.4.7 Deep Learning in Mapping

With the recent interest in novel view synthesis using neural radiance fields (NeRFs) (see Chapter 14 and [764]) —which provides compelling results for image generation via a simple multi-layer perceptron (MLP)— several approaches investigated the usage of neural representations to estimate a SDF. Learning to predict a SDF at arbitrary spatial locations leads to a continuous representation that can be turned into meshes at arbitrary resolutions, but can also lead to more complete representations due to the interpolation capabilities of the learned function.

Similar to implicit representations, this representation is learned from input data and approximated to provide a continuous function that can be queried at arbitrary locations. While often these neural representations are learned ofline with given poses, there has been recent interest towards incremental approaches [1047, 1293] and approaches that estimate poses on-the-fly using a neural representation [1047, 968].

In particular, the approach of Sucar et al. [1047] uses a neural network to predict the SDF value of an arbitrary point in the scene based on RGB-D frames. Follow-up approaches extended this approach by separating the spatial representation of the features via voxel grids [862, 1309], octrees [1293], points [968, 261], etc. from the neural representation. In these approaches, small but descriptive features are stored in a spatial representation and used to determine with a small, neural network the SDF value of an arbitrary point in the scene. This allows decoupling the learned function from the spatial representation, which makes it possible to rely on small neural decoders to turn features into signed distance values, but also being efective for large-scale scenes, such as outdoor environments.

The area of deep learning-based mapping, reconstruction, and SLAM is rapidly evolving and integrates ideas from classical representations, such as surfel splatting [548, 740], to achieve remarkable results in terms of reconstruction quality and eficiency. Recent studies on feed-forward networks, such as Dust3r, for dense representations from monocular video will be illustrated in Chapter 13. In particular, the ability to render novel views and generate new data at arbitrary positions could be potentially exploited for robot learning without relying on simulated environments. For example, NeRF and Gaussian splatting [1318, 103] have gained considerable popularity, demonstrating significant potential in various SLAM-related works. These will be further detailed in Chapter 14.

## 5.5 Usage Considerations

All map representations trade of distinctive, often complementary, strengths and weaknesses. When choosing a map representation for a given application or robotic system, it is therefore important to carefully consider how the map will be used in all downstream tasks. Further factors to consider are the operating environment and available sensors. We will start by discussing environmental factors, which motivate several clear-cut choices, followed by more nuanced task-dependent considerations. Finally, we conclude this chapter with a brief discussion on usage considerations related to the existing methods presented in Section 5.4.

## 5.5.1 Environmental Aspects

Operating environments can be categorized as either structured or unstructured. In tightly controlled spaces, such as automated factories, custom map representations —tailored to the robot’s task and specific objects it will encounter— typically outperform general dense representations in terms of eficiency and accuracy. In contrast, the dense representations covered in this chapter can model objects of arbitrary shapes and work in any environment. When operating in changing or partially unknown environments, it is often important for robots to be able to distinguish observed free space from unobserved space. This information allows path planners to avoid unsafe motions through unobserved space, which could be occupied, and can also be used for exploration planning. Explicit surface representations, including points, surfels, and meshes, generally cannot distinguish between free and unobserved space, while all occupancy-based methods do. Implicit surface-based methods can also provide this distinction, though very often for more reconstruction-focused applications, this information is discarded farther from the surface to save computational and memory costs.

Another consideration is scalability. Explicit representations tend to be more memory eficient than implicit representations, as they only describe the surface itself and their fidelity can easily be adapted to the detail required for each part of the scene. When free-space information is required, multi-resolution approaches can ofer significant improvements over fixed-resolution voxelized representations in terms of accuracy, memory, and their ability to capture very thin objects.

One final consideration is whether the environment has a significant amount of dynamic objects and the degree to which these should be modeled. From the perspective of map representations, most existing approaches can be grouped into one of three categories. The first set of approaches does not consider dynamics and directly fuses all measurements into one of the representations introduced in this chapter. In practice, this might already sufice when using implicit representations, since their free-space updates typically do a good job at erasing leftovers of objects after they moved. The second category of approaches tries to only integrate the environment’s static elements into the map, by explicitly detecting and discarding all measurements corresponding to dynamic objects. This approach is particularly popular when using explicit maps, where leftovers are more tedious to remove, and generally makes it possible to generate clean maps even in highly dynamic spaces. The last set of solutions not only represents the background but also the moving elements in the scene. Note that this is commonly done using hybrid representations, mixing fundamental geometric representations introduced in this chapter with bespoke representations at the object level. For a detailed discussion on SLAM in dynamic environments, including concrete methods to implement the above and more advanced approaches, we refer the reader to Chapter 15.

## 5.5.2 Downstream Task Types

In addition to the environment, it is equally important to consider what map information is necessary for the robot’s required tasks. While any given operation can typically be performed on all representations, the eficiency and implementation complexity tend to vary greatly. The biggest diference lies in whether the operation is performed along the surface or in Cartesian space. As shown in Table 5.2, implicit representations generally allow for simple, eficient filtering of properties that are expressed in Cartesian coordinates, such as occupancy. In contrast, explicit representations are well suited to filter properties that are expressed along the surface, such as visual textures. This explains why explicit representations are generally more sensitive to the quality of the depth measurements, but can create very detailed, visually appealing 3D reconstructions. On the other hand, implicit methods are well suited for fusing noisy depth measurements, such as RGB-D camera data.

In terms of queries, explicit representations make it possible to directly iterate over the surface. This explains their popularity in rendering and graphics applications, and for tasks such as coverage path planning. However, they require additional steps, such as nearest neighbor lookups, to answer queries in Cartesian coordinates. The exact opposite is true for implicit representations, which are therefore commonly used for collision checking tasks.

5.5 Usage Considerations

<table><tr><td rowspan="2">Operation</td><td colspan="2">Efficient in</td></tr><tr><td>Explicit representation</td><td>Implicit representation</td></tr><tr><td>Filter measurements</td><td>Along the surface (texture,...)</td><td>In Cartesian space (occupancy,...)</td></tr><tr><td>Query and iterate</td><td>In surface coordinates (coverage planning,...)</td><td>In Cartesian coordinates (collision checking,...)</td></tr><tr><td>Modify surface</td><td>Geometry (deformation,...)</td><td>Topology (merge, cut, simplify,...)</td></tr></table>

Table 5.2 Complementary strengths and weaknesses of explicit and implicit surface representations.

Finally, explicit representations allow for eficient modifications of the surface’s geometry, including deformations. In practice, maps are often constructed by integrating depth measurements using pose estimates from an imperfect, drifting odometry system. Over time, the accumulated pose errors also lead to inconsistencies in the dense map. Just like in SLAM systems, these errors can be eliminated by deforming the dense map when detecting loop closures. Although both explicit and implicit surfaces can be deformed, this operation is inherently simpler and more eficient when using an explicit representation. In contrast, using an implicit representation simplifies and improves the eficiency of operations afecting the surface’s topology, or connectivity. Implicit representations are therefore often used to merge surface estimates, combine or subtract object shapes, and simplify surfaces.

It is important to remember that diferent representations can also be used in tandem to leverage their respective strengths. One good example of a hybrid approach is TSDF-based meshing (Section 5.4.3), where noisy depth measurements are first conveniently filtered using an implicit surface representation (TSDF) which is then converted to an explicit representation (mesh) using Marching Cubes. When deciding whether the advantages of hybrid representations outweigh the overhead they introduce, it is worth considering how the conversions can be limited to only happen locally and infrequently.

## 5.5.3 Summary of Mapping Methods

We now conclude our discussion by summarizing the key diferences between the existing methods presented in this chapter. Starting with the explicit representations, using a collection of points to describe the surface is simple and requires the fewest assumptions, but it is also the least informative. Beyond infinitesimal points, surfels represent the surface’s properties over small neighborhoods, or patches. Finally, meshes explicitly represent the surface’s connectivity and allow its properties to smoothly be interpolated. However, estimating the surface’s connectivity requires the most assumptions and sometimes comes at a significant computational cost.

In terms of implicit representations, a particular advantage of implicit surfaces over occupancy maps is that they ofer fast, high-quality distance information and gradients which are beneficial for optimization-based planning. However, filtering occupancy estimates requires less assumptions and, for voxel-based methods, occupancy maps are better at capturing thin obstacles. In cases with particularly noisy or sparse depth measurements, non-voxelized implicit representations, based on GPs and Hilbert Maps, provide particularly good uncertainty estimates. As they explicitly consider the geometry’s spatial correlations, they are generally also better at interpolating partially observed surfaces.

One rapidly advancing research area is that of learning-based methods. In terms of learning-based implicit representations, NeRFs have been shown to enable promising new capabilities, particularly for semantic modeling and spatial reasoning. More recently, Gaussian splatting [552] —an explicit representation bearing similarities to surfels— led to an increasing interest into approaches using splatting [548, 740, 1261]. Researchers are actively working on improving the computational and memory footprint of these approaches, testing what new skills they can enable, and exploring how they can be integrated into complete robotic systems. Looking ahead, we expect learning-based methods to increase the generality and expressiveness of dense representations, while improving their ability to handle noisy measurements, incomplete observations, and dynamic objects through learned priors.

## Acknowledgments

The authors thank Lan Wu for her support in preparing this chapter.

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