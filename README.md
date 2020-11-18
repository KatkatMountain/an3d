# AN3D - TPs

> Simon REGOURD & Kathialina VA

Ce repository contient le code source des TPs du cours d'AN3D concernés dans le compte-rendu : *3.spheres in collisions* et *4.Cloth simulation*.
Il contient donc également la bibliothèque VCL et le code fourni par M.Damien ROHMER.

Ce README contient notamment images animées de démonstration, en complément de compte-rendu. (Les GIFS peuvent contenir des artefacts de par leur compression...)

## Usage
1. Choisir la scène à charger en commentant/décommentant les `#define` dans `scene/current_scene.hpp`.
2. `mkdir build`
3. `cd build`
4. `cmake ..`
5. `make`
6. Lancer le programme à partir du dossier parent.

## TP : Sphères en collision
### Standard
![](https://i.imgur.com/TlpKuTP.gif)

### Gravité selon la caméra
![](https://i.imgur.com/uVfb0gg.gif)


### Une sphère pour contenant
![](https://i.imgur.com/STSPHzS.gif)

## TP : Simulation de tissu
### Standard
![](https://i.imgur.com/LAupXuk.gif)

### Avec du vent
![](https://i.imgur.com/gHy2myw.gif)
