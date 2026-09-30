# Série de vidéos biorbd

Courtes vidéos Manim (environ 35 s chacune, en anglais et en français) qui expliquent le fonctionnement de biorbd. Chaque nombre
affiché vient d'un vrai calcul biorbd 1.12.3 ; les données synthétiques et les algorithmes écrits à la main sont étiquetés comme
tels dans les vidéos. English: [README.md](README.md).

Les vidéos ne sont pas stockées dans le dépôt (elles sont rendues en local, voir ci-dessous).

## Commencer ici

1. **Modèles** : 01 anatomie d'un `.bioMod`, puis 02 chargement d'un modèle.
2. **Cinématique** : 03 cinématique directe, 04 rotations, 05 jacobiennes.
3. **Cinématique inverse** : 15 à la main, puis 14 avec `biorbd.InverseKinematics`, puis 16 avec le filtre de Kalman.
4. **Masse et dynamique** : 06 centre de masse, 07 matrice de masse, 08 effets non linéaires, 09 dynamique inverse, 10 dynamique directe.
5. **Muscles** : 11 muscles, puis 17 optimisation statique.
6. **Contacts, capteurs, CasADi** : 12 contacts, 13 IMU, 18 CasADi.

## Les vidéos

<!-- TABLE:BEGIN -->
| # | Vidéo | Ce qu'elle montre | Niveau | Fichier de scène | Exemples de code |
|---|---|---|---|---|---|
| 01 | **Anatomie d'un fichier .bioMod** | Un fichier .bioMod est du texte brut : les balises segment donnent les coordonnées q, la masse et le centre de masse, les balises marker donnent des points fixes dans un segment, et biorbd les relit. | 1 (bases) | [`anim_biomod_anatomy.py`](anim_biomod_anatomy.py) | [`README.md`](../../README.md), [`README.md`](../../README.md), [`pyomecaman.bioMod`](../../examples/pyomecaman.bioMod), [`ModelReader.cpp`](../../src/ModelReader.cpp), [`ModelReader.cpp`](../../src/ModelReader.cpp) |
| 02 | **Charger un modèle** | biorbd.Biorbd(chemin) lit un fichier .bioMod et renvoie un objet modèle dont on peut lister les degrés de liberté, les segments, les marqueurs et les muscles. | 1 (bases) | [`anim_model_loading.py`](anim_model_loading.py) | [`forward_dynamics_from_muscles.py`](../../examples/python3/forward_dynamics_from_muscles.py), [`biorbd_model.py`](../../binding/python3/wrapper/biorbd_model.py), [`biorbd_model.py`](../../binding/python3/wrapper/biorbd_model.py), [`arm26.bioMod`](../../examples/arm26.bioMod) |
| 03 | **Cinématique directe** | À partir des coordonnées généralisées q, biorbd enchaîne les repères des segments d'une jambe et donne la position d'un marqueur dans le monde. | 1 (bases) | [`anim_forward_kinematics.py`](anim_forward_kinematics.py) | [`forward_kinematics.py`](../../examples/python3/forward_kinematics.py), [`marker.py`](../../binding/python3/wrapper/marker.py), [`segment_frame.py`](../../binding/python3/wrapper/segment_frame.py), [`Joints.h`](../../include/RigidBody/Joints.h) |
| 04 | **Rotations** | Les trois mêmes angles d'Euler donnent des matrices de rotation différentes dans les séquences xyz et zyx ; biorbd convertit entre angles, matrices et quaternions. | 2 | [`anim_rotations.py`](anim_rotations.py) | [`Rotation.h`](../../include/Utils/Rotation.h), [`Rotation.h`](../../include/Utils/Rotation.h), [`Quaternion.h`](../../include/Utils/Quaternion.h), [`segment_frame.py`](../../binding/python3/wrapper/segment_frame.py), [`manipulating_frames_of_reference.py`](../../examples/python3/manipulating_frames_of_reference.py) |
| 05 | **Jacobiennes de marqueurs** | La jacobienne d'un marqueur relie les vitesses généralisées à la vitesse du marqueur dans le repère monde, v = J(q) qdot. La vidéo montre la petite matrice d'un marqueur de l'avant-bras du modèle de bras et vérifie le produit par des différences finies des positions du marqueur le long d'une courte trajectoire. | 2 | [`anim_jacobians.py`](anim_jacobians.py) | [`marker.py`](../../binding/python3/wrapper/marker.py), [`Markers.h`](../../include/RigidBody/Markers.h), [`test_wrapper_markers.py`](../../test/binding/python3/tests_wrapper/test_wrapper_markers.py), [`forward_kinematics.py`](../../examples/python3/forward_kinematics.py) |
| 06 | **Centre de masse** | Le centre de masse du corps entier est la moyenne pondérée par les masses des centres de masse des segments, et il se déplace quand la posture q change. | 1 (bases) | [`anim_center_of_mass.py`](anim_center_of_mass.py) | [`manipulation_model.py`](../../examples/python3/manipulation_model.py), [`biorbd_model.py`](../../binding/python3/wrapper/biorbd_model.py), [`segment.py`](../../binding/python3/wrapper/segment.py), [`Joints.h`](../../include/RigidBody/Joints.h) |
| 07 | **La matrice de masse** | La matrice de masse M(q) dans l'espace articulaire donne la part inertielle des couples articulaires, M(q) fois les accélérations. Elle est symétrique, définie positive et dépend de la posture. La vidéo calcule M pour le modèle arm26 à deux angles du coude, vérifie la symétrie et la plus petite valeur propre, puis montre comment la matrice change d'une posture à l'autre. | 2 | [`anim_mass_matrix.py`](anim_mass_matrix.py) | [`biorbd_model.py`](../../binding/python3/wrapper/biorbd_model.py), [`Joints.h`](../../include/RigidBody/Joints.h), [`test_wrapper_model.py`](../../test/binding/python3/tests_wrapper/test_wrapper_model.py), [`arm26.bioMod`](../../examples/arm26.bioMod) |
| 08 | **Effets non linéaires** | model.non_linear_effect(q, qdot) renvoie les forces généralisées qui annulent la gravité et les termes dépendant de la vitesse (Coriolis et centrifuge). À vitesse nulle, c'est la gravité seule ; la partie dépendant de la vitesse croît avec le carré de la vitesse. La vidéo utilise le modèle pyomecaman dans une posture choisie à la main : la force verticale du bassin vaut la masse fois 9,81, et doubler qdot multiplie exactement par quatre la partie dépendant de la vitesse. biorbd n'a pas de fonction Coriolis séparée. | 2 | [`anim_nonlinear_effects.py`](anim_nonlinear_effects.py) | [`biorbd_model.py`](../../binding/python3/wrapper/biorbd_model.py), [`Joints.h`](../../include/RigidBody/Joints.h), [`Joints.cpp`](../../src/RigidBody/Joints.cpp), [`test_wrapper_model.py`](../../test/binding/python3/tests_wrapper/test_wrapper_model.py) |
| 09 | **Dynamique inverse** | La dynamique inverse donne les couples articulaires qui produisent un mouvement donné (q, qdot, qddot). La vidéo les calcule avec model.inverse_dynamics sur le modèle de bras et vérifie qu'ils valent M(q) qddot + N(q, qdot), avec la matrice de masse et les effets non linéaires de biorbd. | 2 | [`anim_inverse_dynamics.py`](anim_inverse_dynamics.py) | [`inverse_dynamics.py`](../../examples/python3/inverse_dynamics.py), [`biorbd_model.py`](../../binding/python3/wrapper/biorbd_model.py), [`biorbd_model.py`](../../binding/python3/wrapper/biorbd_model.py), [`biorbd_model.py`](../../binding/python3/wrapper/biorbd_model.py) |
| 10 | **Dynamique directe et intégration** | model.forward_dynamics renvoie les accélérations qddot à partir de q, qdot et tau. biorbd n'a pas d'intégrateur dans son cœur : un Euler et un RK4 écrits à la main sont comparés sur un double pendule passif grâce à la dérive de son énergie totale. | 2 | [`anim_forward_dynamics.py`](anim_forward_dynamics.py) | [`forward_dynamics.py`](../../examples/python3/forward_dynamics.py), [`biorbd_model.py`](../../binding/python3/wrapper/biorbd_model.py), [`Joints.h`](../../include/RigidBody/Joints.h), [`pendulum.bioMod`](../../test/models/pendulum.bioMod) |
| 11 | **Muscles : longueur, bras de levier et couple** | La longueur d'un muscle dépend des angles articulaires, et le gradient de cette longueur par rapport aux coordonnées est la jacobienne de longueur. Le couple articulaire produit par les muscles est l'opposé de la jacobienne de longueur transposée fois les forces musculaires. La vidéo balaie l'angle du coude du modèle de bras, montre la longueur et le bras de levier du long chef du biceps, et compare le couple renvoyé par muscles.joint_torque à moins J transposée fois F calculé à la main. | 2 | [`anim_muscles.py`](anim_muscles.py) | [`manipulating_muscles.py`](../../examples/python3/manipulating_muscles.py), [`muscle.py`](../../binding/python3/wrapper/muscle.py), [`muscle.py`](../../binding/python3/wrapper/muscle.py), [`Muscles.h`](../../include/InternalForces/Muscles/Muscles.h), [`test_wrapper_muscles.py`](../../test/binding/python3/tests_wrapper/test_wrapper_muscles.py) |
| 12 | **Contacts rigides** | Un contact rigide déclaré dans le fichier bioMod interdit le mouvement de son point selon les axes choisis. La vidéo calcule la dynamique directe d'un cube libre muni de deux contacts, d'abord en ignorant les contacts (chute libre), puis avec eux : les accélérations généralisées s'annulent, l'accélération des points de contact est nulle à la précision numérique et biorbd renvoie les forces de contact, dont les composantes verticales s'additionnent pour donner le poids. | 3 (avancé) | [`anim_contacts.py`](anim_contacts.py) | [`biorbd_model.py`](../../binding/python3/wrapper/biorbd_model.py), [`Contacts.h`](../../include/RigidBody/Contacts.h), [`cubeWithRigidContactsExternalForces.bioMod`](../../test/models/cubeWithRigidContactsExternalForces.bioMod), [`test_rigidbody.cpp`](../../test/test_rigidbody.cpp) |
| 13 | **Centrale inertielle (IMU)** | Une IMU déclarée dans le .bioMod est un repère fixé dans un segment : biorbd donne son orientation dans le monde à partir de q. | 3 (avancé) | [`anim_imu.py`](anim_imu.py) | [`pyomecaman_withIMUs.bioMod`](../../test/models/IMUandCustomRT/pyomecaman_withIMUs.bioMod), [`IMUs.h`](../../include/RigidBody/IMUs.h), [`IMUs.cpp`](../../src/RigidBody/IMUs.cpp), [`test_conversion.py`](../../test/binding/python3/test_conversion.py) |
| 14 | **La cinématique inverse par moindres carrés** | biorbd.InverseKinematics résout chaque image avec un solveur de moindres carrés de scipy (lm, trf ou only_lm), ignore les marqueurs NaN et, avec trf, respecte les amplitudes articulaires. Montré sur des marqueurs synthétiques bruités d'un modèle à 13 ddl, avec l'erreur mesurée par rapport à la vérité connue. | 2 | [`anim_ik_least_squares.py`](anim_ik_least_squares.py) | [`rigid_body.py`](../../binding/python3/rigid_body.py), [`rigid_body.py`](../../binding/python3/rigid_body.py), [`utils.py`](../../binding/python3/utils.py), [`test_inverse_kinematics.py`](../../test/binding/python3/test_inverse_kinematics.py), [`pyomecaman.bioMod`](../../examples/pyomecaman.bioMod) |
| 15 | **La cinématique inverse à la main** | Une boucle de Gauss-Newton écrite à la main retrouve les angles articulaires d'un bras à deux degrés de liberté à partir de marqueurs synthétiques bruités, avec la jacobienne des marqueurs de biorbd. | 2 | [`anim_ik_by_hand.py`](anim_ik_by_hand.py) | [`marker.py`](../../binding/python3/wrapper/marker.py), [`forward_kinematics.py`](../../examples/python3/forward_kinematics.py), [`rigid_body.py`](../../binding/python3/rigid_body.py), [`test_inverse_kinematics.py`](../../test/binding/python3/test_inverse_kinematics.py) |
| 16 | **Filtre de Kalman** | Le filtre de Kalman étendu de biorbd reconstruit les angles articulaires image par image à partir de marqueurs synthétiques bruités, et il est comparé à la résolution de chaque image seule. | 3 (avancé) | [`anim_kalman_filter.py`](anim_kalman_filter.py) | [`extended_kalman_filter.py`](../../binding/python3/wrapper/extended_kalman_filter.py), [`KalmanRecons.h`](../../include/RigidBody/KalmanRecons.h), [`inverseKinematicsKalmanExample.cpp`](../../examples/inverseKinematicsKalmanExample.cpp), [`test_wrapper_kalman_filter.py`](../../test/binding/python3/tests_wrapper/test_wrapper_kalman_filter.py) |
| 17 | **L'optimisation statique** | Six muscles agissent sur deux articulations : de nombreux jeux d'activations donnent le même couple articulaire. L'optimisation statique garde celui dont la somme des activations au carré est la plus petite, et le couple musculaire reproduit la cible de dynamique inverse. | 3 (avancé) | [`anim_static_optimization.py`](anim_static_optimization.py) | [`static_optimization.py`](../../binding/python3/wrapper/static_optimization.py), [`static_optimization.py`](../../examples/python3/static_optimization.py), [`muscle.py`](../../binding/python3/wrapper/muscle.py), [`test_wrapper_muscles.py`](../../test/binding/python3/tests_wrapper/test_wrapper_muscles.py), [`StaticOptimization.cpp`](../../src/InternalForces/Muscles/StaticOptimization.cpp) |
| 18 | **Fonctions symboliques avec la version CasADi** | La version CasADi de biorbd (biorbd_casadi, installée séparément de la version Eigen par défaut) transforme toute grandeur du modèle en fonction symbolique CasADi avec to_casadi_func. La vidéo construit la matrice de masse d'un pendule à deux segments en fonction de q, montre un terme symbolique et compare sa valeur en une posture avec la matrice de masse de la version par défaut. | 3 (avancé) | [`anim_casadi_symbolic.py`](anim_casadi_symbolic.py) | [`__init__.py`](../../binding/python3/__init__.py), [`test_binder_python_rigidbody.py`](../../test/binding/python3/test_binder_python_rigidbody.py), [`biorbd_model.py`](../../binding/python3/wrapper/biorbd_model.py), [`Joints.h`](../../include/RigidBody/Joints.h) |
<!-- TABLE:END -->

Le tableau est généré depuis [`catalog.json`](catalog.json) par `tools/make_readme_table.py` (ne pas le modifier à la main).

## Prérequis

- **Rendu** : Python 3.11 avec `manim` 0.21 (Community Edition). Polices utilisées : Segoe UI et Consolas (Windows) ; sur un autre
  système, il faut ces polices, sinon les polices de repli changent la mise en page.
- **Génération des données** (`generate_*_data.py`) : un environnement conda avec biorbd 1.12.3 de conda-forge (Python 3.12 ; le
  wrapper de haut niveau `biorbd.Biorbd` demande Python >= 3.12), numpy et scipy. Le paquet conda fournit les deux backends :
  Eigen (`biorbd`) et CasADi (`biorbd_casadi`). Sous Windows, préférez OpenBLAS à MKL si `numpy.linalg` plante.
- Les fichiers `data/*.npz` versionnés suffisent pour rendre les vidéos sans biorbd.
- Le logo est téléchargé, pas versionné : `python tools/fetch_logo.py`.

## Commandes

```bash
python tools/fetch_logo.py                                  # une fois
python tools/render_series.py --list                        # lister les scènes
python tools/render_series.py AnimJacobians --lang both --quality 480p15   # brouillon rapide
python tools/render_series.py --all --lang both --jobs 3 --strict --collect
python tools/extract_frames.py media/AnimJacobians_fr/videos/anim_jacobians/1080p30/AnimJacobians.mp4
python tools/validate_catalog.py
python tools/make_readme_table.py --check
```

`--strict` échoue sur tout constat d'audit des cadres (texte hors cadre, recouvrement, collision avec le logo), sur une clé de
traduction française manquante ou sur une entrée de catalogue manquante. `--collect` copie les vidéos dans
`<Documents>/biorbd_animations/EN` et `/FR` (modifiable avec `--out` ou `BIORBD_ANIM_OUT`). Règles pour de nouvelles vidéos :
[STANDARD.fr.md](STANDARD.fr.md) et `templates/scene_template.py`.

## Avertissements honnêtes

- L'audit des cadres ne voit pas tout (il ne s'exécute qu'entre les animations) : il faut quand même regarder les images.
- Les scènes utilisent de petits modèles, souvent plans (surtout `arm26`, parfois `pyomecaman`). Les nombres illustrent la
  méthode ; ils ne valident pas biorbd sur d'autres modèles, des cas 3D ou des mesures réelles.
- La vidéo 17 contourne `StaticOptimization.perform_frames` du wrapper, qui appelle le solveur linéarisé et ne reproduisait pas le
  couple cible dans notre test ; voir `notes/static_optimization.md`.
- Certaines vidéos utilisent un intégrateur ou un solveur écrit à la main (10, 15) à des fins pédagogiques ; biorbd ne contient
  pas d'intégrateur RK.
- Chaque scène a un fichier `notes/<sujet>.md` qui liste ce qui a été vérifié et ce qui ne l'a pas été.
