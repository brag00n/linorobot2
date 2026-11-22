/* partmaint.catalog.js : CATALOGUE du slide « Maintenance d'une Part » (2026-11-22). Donnees declaratives uniquement, aucun code d'interface.

   Chaque entree est indexee par l'identifiant d'une part de parts.yaml. Une part absente d'ici n'a ni action ni metrique propre : le slide affiche quand meme
   les valeurs de ses taches de diagnostic (`diag:` de parts.yaml) et sa documentation.

   Une ACTION :
     id          identifiant stable (journal, tests)
     label       texte du bouton
     service     service ROS appele par rosbridge (doit etre dans la liste blanche de docker/rosbridge/start.sh)
     type        type du service (documentation, rosbridge le deduit)
     args        arguments de l'appel
     risque      "faible" : un clic suffit apres l'avoir lu ; "moyen" / "eleve" : popup de validation, appui maintenu 3 s
     decrit      ce que fait l'action, en clair
     effet       ce que le robot fait ou subit (affiche dans la popup)
     requiert    conditions evaluees AVANT d'autoriser le bouton : "arret" (traction desarmee), "liaison" (CC et video sains)
     lecture     vrai : pas d'effet (jamais de popup)
   Le robot refuse AUSSI ce que la page autorise : la page n'est jamais la seule barriere.

   Une METRIQUE hors diagnostic :
     id, label, unite, source ("V" = variable de la page, "EN" = energie), cle, courbe (vrai : mini-courbe)

   Limite connue : relancer un service Docker n'a pas de chemin sur le robot aujourd'hui (le conteneur n'a pas le socket Docker) : aucune action de ce genre
   ici, voir le plan, incrément 2. */
(function (root) {
  "use strict";

  var ACTIONS = {
    imu_link: [
      { id: "recalibrer_gyro", label: "Recalibrer le gyroscope", service: "/esp32_mavlink_driver/recalibrate_imu", type: "std_srvs/srv/Trigger", args: {},
        risque: "moyen", requiert: ["arret", "liaison"],
        decrit: "La carte moyenne le biais du gyroscope pendant environ 2 s.",
        effet: "Le robot doit rester STRICTEMENT immobile pendant la mesure : un mouvement fausse le zéro du gyroscope et le cap dérive ensuite." }
    ],
    laser: [
      { id: "pile_nav_on", label: "Démarrer la pile de navigation", service: "/bamboo/nav_stack", type: "std_srvs/srv/SetBool", args: { data: true },
        risque: "moyen", requiert: ["liaison"],
        decrit: "Démarre le lidar et la navigation (Nav2, SLAM) via le groupe à la demande.",
        effet: "Charge CPU forte (jusqu'à 60 % supplémentaires pendant le démarrage). Le robot ne bouge pas tant qu'aucun but n'est donné." },
      { id: "pile_nav_off", label: "Arrêter la pile de navigation", service: "/bamboo/nav_stack", type: "std_srvs/srv/SetBool", args: { data: false },
        risque: "eleve", requiert: ["liaison"],
        decrit: "Arrête le lidar et la navigation immédiatement.",
        effet: "Annule tout but en cours. La carte en cours n'est pas sauvegardée par ce bouton : sauvegarder avant (manette, A long en NAV)." }
    ],
    navigation: [
      { id: "pile_nav_on", label: "Démarrer la pile de navigation", service: "/bamboo/nav_stack", type: "std_srvs/srv/SetBool", args: { data: true },
        risque: "moyen", requiert: ["liaison"],
        decrit: "Démarre le lidar et la navigation (Nav2, SLAM) via le groupe à la demande.",
        effet: "Charge CPU forte pendant le démarrage. Le robot ne bouge pas tant qu'aucun but n'est donné." },
      { id: "pile_nav_off", label: "Arrêter la pile de navigation", service: "/bamboo/nav_stack", type: "std_srvs/srv/SetBool", args: { data: false },
        risque: "eleve", requiert: ["liaison"],
        decrit: "Arrête le lidar et la navigation immédiatement.",
        effet: "Annule tout but en cours. La carte en cours n'est pas sauvegardée par ce bouton." },
      { id: "annuler_but", label: "Annuler le but de navigation", service: "/nav_supervisor/command", type: "bamboo_interfaces/srv/NavCommand", args: { command: "cancel", arg: "" },
        risque: "faible", requiert: ["liaison"],
        decrit: "Annule le but en cours et arrête le déplacement.",
        effet: "Le robot s'arrête. Sans effet s'il n'y a pas de but." }
    ],
    cp2102_lidar: [
      { id: "pile_nav_on", label: "Démarrer la pile de navigation", service: "/bamboo/nav_stack", type: "std_srvs/srv/SetBool", args: { data: true },
        risque: "moyen", requiert: ["liaison"],
        decrit: "Redémarre le lidar et la navigation après un rebranchement : le pilote ne reprend pas seul.",
        effet: "Charge CPU forte pendant le démarrage. À faire APRÈS avoir rebranché le lidar à la main (jamais de reset logiciel du hub USB : il est partagé avec l'ESP32)." }
    ],
    bamboo_control: [
      { id: "desarmer", label: "Désarmer la traction", service: "/cc/guard/arm", type: "std_srvs/srv/SetBool", args: { data: false },
        risque: "faible", requiert: ["liaison"],
        decrit: "Coupe la traction. Toujours accepté.",
        effet: "Le robot s'arrête et ne répond plus à la manette jusqu'au prochain armement." },
      { id: "armer", label: "Armer la traction", service: "/cc/guard/arm", type: "std_srvs/srv/SetBool", args: { data: true },
        risque: "eleve", requiert: ["liaison"],
        decrit: "Arme la traction : le robot peut rouler dès que la manette ou un but le commande.",
        effet: "LE ROBOT PEUT SE DÉPLACER. Roues levées ou zone dégagée exigées. Le garde-fou du CC désarme si la liaison ou la vidéo se perd." },
      { id: "armer_verrouille", label: "Armer verrouillé", service: "/cc/guard/arm_locked", type: "std_srvs/srv/SetBool", args: { data: true },
        risque: "eleve", requiert: ["liaison"],
        decrit: "Arme la traction ; la perte de la manette ne désarme plus (le garde-fou du CC désarme toujours).",
        effet: "LE ROBOT PEUT SE DÉPLACER sans manette connectée. Zone dégagée exigée." }
    ]
  };

  /* Metriques propres (hors taches de diagnostic) : lues dans les variables de la page. */
  var METRICS = {
    batterie: [
      { id: "bat_v", label: "Tension", unite: "V", source: "V", cle: "bat_v", courbe: true },
      { id: "bat_soc", label: "Charge", unite: "%", source: "V", cle: "soc", courbe: true },
      { id: "bat_a", label: "Courant", unite: "A", source: "V", cle: "cur", courbe: true }
    ],
    imu_link: [
      { id: "roll", label: "Roulis", unite: "°", source: "V", cle: "roll", courbe: true },
      { id: "pitch", label: "Tangage", unite: "°", source: "V", cle: "pitch", courbe: true },
      { id: "yaw", label: "Cap", unite: "°", source: "V", cle: "yaw", courbe: true }
    ],
    rpi: [
      { id: "cpu", label: "CPU", unite: "%", source: "V", cle: "cpu", courbe: true },
      { id: "temp", label: "Température", unite: "°C", source: "V", cle: "temp", courbe: true },
      { id: "mem", label: "Mémoire", unite: "%", source: "V", cle: "mem", courbe: true }
    ]
  };

  var catalog = { actions: ACTIONS, metrics: METRICS, RISQUES: ["faible", "moyen", "eleve"], REQUIERT: ["arret", "liaison"] };
  root.PartMaintCatalog = catalog;
  if (typeof module !== "undefined" && module.exports) module.exports = catalog;
})(typeof window !== "undefined" ? window : this);
