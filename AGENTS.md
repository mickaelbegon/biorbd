# Validation des bindings Python de biorbd

Ces consignes complètent les préférences globales pour ce dépôt C++ et ses
bindings Eigen et CasADi.

- Compiler la cible et le binding concernés avant les tests. Utiliser le même
  interpréteur Python que celui choisi à la configuration du build ; un module
  natif compilé pour une autre version de Python peut provoquer un segfault à
  l'import.
- Vérifier que `PYTHONPATH` sélectionne le package fraîchement compilé pour le
  backend testé. Éviter d'y mélanger des packages Eigen et CasADi construits avec
  des ABI Python différentes. Pour un package de staging, synchroniser aussi le
  wrapper Python généré avec le module natif.
- Terminer le build avant de lancer un processus Python. Ne pas relinker les
  bibliothèques pendant qu'un processus les utilise.
- Lancer les suites Python depuis `test/binding/Python3` : certains tests
  existants résolvent leurs chemins de modèles relativement au répertoire de
  travail.

Pour la suite Eigen, avec le package compatible sur `PYTHONPATH`, depuis la
racine du dépôt :

```sh
cd test/binding/Python3
python -m pytest -q
```

Pour les régressions wrapping CasADi, utiliser l'interpréteur de ce backend et
le module explicite, depuis le même répertoire :

```sh
python -m unittest test_wrapping_casadi -v
```

La découverte récursive `unittest discover` peut importer des sous-packages
comme `de_leva`, qui chargent Eigen même lorsque le motif sélectionne uniquement
les tests CasADi. En présence de builds pour des versions Python différentes,
utiliser la commande explicite ci-dessus. En cas d'échec à l'import, vérifier
l'origine du package et la compatibilité de l'interpréteur avec le module natif.
