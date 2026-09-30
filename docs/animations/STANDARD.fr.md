# Standard des vidéos biorbd

Toutes les vidéos de la série suivent ce document. `STANDARD.md` est la version anglaise ; les deux restent synchronisées.
La couche de rendu (`common/layer.py`) gère déjà le logo, le ralentissement, les transitions de texte, la traduction,
la carte finale et l'audit des cadres : une scène ne contient donc que le contenu.

## 1. Règles de données (D)

- **D1 — Uniquement de vrais calculs.** Chaque nombre, courbe, pose ou trajectoire vient d'un vrai appel biorbd
  fait par un script `generate_<sujet>_data.py` et stocké dans `data/<sujet>_*.npz`. Jamais de résultat tapé à la main.
- **D2 — Les nombres affichés viennent du `.npz`.** Formatez-les dans la scène à partir des tableaux chargés,
  avec un seul gabarit par grandeur (par exemple `f"{value:.2f}"`), jamais en littéraux.
- **D3 — Étiqueter ce qui n'est pas une sortie de biorbd.** Données synthétiques (mouvement simulé, bruit ajouté),
  interfaces redessinées et algorithmes écrits à la main (intégrateur RK4, Gauss-Newton) portent une étiquette
  à l'écran (`bio_style.synthetic_tag`).
- **D4 — Vérifié sur cette version.** Le code et les noms d'API affichés sont vérifiés dans biorbd 1.12.3
  (`grep` dans `include/` et `binding/python3/`). Les générateurs tournent dans l'environnement `biorbd_anim`.
- **D5 — Le code affiché est le code exécuté.** Les lignes du panneau de code sont celles que le générateur exécute
  (copiées, ou lues dans sa source). Pas de pseudo-code raccourci.
- **D6 — Si cela ne marche pas, on s'arrête.** N'inventez pas de résultat : écrivez ce qui a été essayé dans `notes/<sujet>.md`.

## 2. Anatomie d'une scène (A)

- **A1 — Titre et sous-titre** en haut à gauche (`bio_style.title_block`) : titre 44 pt, sous-titre 28 pt.
- **A2 — Visuels à gauche** (x de -6,8 à -0,5). **A3 — Panneau de code à droite** (`bio_style.code_panel`),
  intitulé « Code biorbd », légende **au-dessus** du cadre de code.
- **A4 — Une phrase de conclusion** en bas (`bio_style.footer`), une phrase entière.
- **A5 — Le contenu dure 10 à 20 s à vitesse native et développe 2 idées au plus.** La couche ralentit tout
  d'un facteur 1,6, garde la dernière image 2,5 s, puis affiche la carte « Pour aller plus loin » de 4,5 s.
  N'ajoutez ni pause finale ni carte de fin.
- **A6 — Quand une grandeur change, tracez une courbe fantôme et un axe de différence :** gardez la courbe
  précédente estompée (opacité 0,3) et montrez la différence sur un petit axe en dessous, en `C_DIFF`.
- **A7 — Les rôles de couleur sont fixes** (voir `bio_style.py`) : `C_TITLE` l'objet expliqué, `C_MODEL` ce que biorbd
  calcule, `C_DATA` données d'entrée ou de référence, `C_DIFF` différences, `C_SYNTH` étiquettes synthétiques,
  `C_OK` vérifications réussies. Polices : `Segoe UI` pour le texte, `Consolas` pour le code (posées par la couche).

## 3. Règles de texte (T)

- **T1 — Phrases entières**, une idée chacune. **T2 — Pas de `Paragraph`, pas de LaTeX / `Tex` / `MathTex`.**
  Écrivez les formules en `Text`.
- **T3 — Les nombres suivent un gabarit unique** par scène pour que la virgule décimale française soit
  appliquée uniformément (la couche s'en charge).
- **T4 — Évitez `t2c`, les tranches et la coloration dans une chaîne traduisible** : la traduction change la
  position des caractères. Utilisez plusieurs objets `Text`.
- **T5 — Prévoir la place du français** : environ 20 % plus long. Ne dimensionnez pas les cadres sur l'anglais.
- **T6 — Le texte en police de code n'est jamais traduit.** Utilisez `font=bio_style.CODE_FONT` pour le code,
  les noms et les chemins.
- **T7 — `Write`, `FadeIn`, `ReplacementTransform` sont utilisables** : la couche transforme Write/lettre par
  lettre en fondu, et un morph texte vers texte en fondu sortant puis entrant, sans chevauchement.
- **T8 — Ne touchez pas au logo.** Pas de `self.clear()`, ne le faites pas disparaître ; estompez les objets
  un à un ou dans un `Group` qui l'exclut.

## 4. Fichiers

| Fichier | Rôle |
|---|---|
| `anim_<sujet>.py` | une classe de scène par fichier, nommée `Anim<Sujet>` (CamelCase) |
| `generate_<sujet>_data.py` | écrit `data/<sujet>_*.npz` |
| `data/<sujet>_*.npz` | versionné (petit) |
| `notes/<sujet>.md` | ce qui a été vérifié, ce qui a échoué, questions ouvertes |
| `i18n/fr_<sujet>.json` | table française de la scène |
| `catalog_part_<sujet>.json` | entrée de catalogue de la scène (fusionnée par `tools/merge_catalog.py`) |

Les `anim__*.py` sont des auto-tests de la couche, hors série. N'écrivez jamais de fichier à la racine du dépôt.

## 5. Entrée de catalogue

Une entrée par scène dans `catalog.json` (voir l'exemple dans `STANDARD.md`) : `id`, `slug`, `scene`, `level` (1 à 3),
`section`, `title`, `description` et `notes` en EN/FR, et 2 à 5 `links` vers des exemples et du code. Chaque `path`
existe et les `lines` sont dans le fichier (contrôlé par `tools/validate_catalog.py` ; trouvez-les avec `grep -n`).

## 6. Flux de traduction

1. Rendre en anglais : `python tools/render_series.py <Scene> --lang en`. Le rapport
   `media/<Scene>_en/report_<Scene>_en.json` liste toutes les clés (`keys`).
2. Écrire `i18n/fr_<sujet>.json` : clé = la chaîne anglaise dont les nombres sont remplacés par `{0}`, `{1}` ;
   valeur = la chaîne française avec les mêmes marqueurs. Utiliser le glossaire `i18n/GLOSSARY.md`.
3. Rendre avec `--lang fr --strict` : une clé manquante fait échouer l'exécution.

## 7. Commandes

```bash
python tools/fetch_logo.py                       # une fois (le PNG n'est pas versionné)
python tools/render_series.py --list
python tools/render_series.py <Scene> --lang both --quality 480p15   # contrôle rapide
python tools/render_series.py --all --lang both --jobs 3 --strict --collect
python tools/extract_frames.py media/<Scene>_fr/videos/anim_<sujet>/1080p30/<Scene>.mp4
python tools/validate_catalog.py
black -t py311 -l120 docs/animations
```

Environnements : le rendu utilise Python 3.11 avec `manim` 0.21 (`envs/manim311`) ; la génération des données
utilise l'environnement conda `biorbd_anim` (Python 3.12, biorbd 1.12.3).

## 8. Liste de contrôle et définition de « terminé »

- [ ] Les données viennent de `.npz` produits par un vrai calcul biorbd (D1-D2) ; le synthétique est étiqueté (D3).
- [ ] Le code affiché est le code exécuté et existe dans cette version (D4-D5).
- [ ] Anatomie respectée (A1-A7), 10 à 20 s de contenu, 2 idées au plus.
- [ ] Règles de texte respectées (T1-T8) ; table française complète.
- [ ] `render_series.py --strict` réussit en EN et FR à 1080p30 (aucun constat d'audit).
- [ ] Images contrôlées avec `extract_frames.py` (environ 1 s, 35 %, 65 %, avant la carte, milieu de la carte) :
      les PNG ont réellement été regardés ; rien hors cadre, en recouvrement ou au contact du logo, même en français.
- [ ] Entrée de catalogue validée. `black -t py311 -l120` lancé après la dernière modification.

Une scène est **terminée** seulement quand toutes les cases sont cochées et que `notes/<sujet>.md` dit ce qui reste non vérifié.
