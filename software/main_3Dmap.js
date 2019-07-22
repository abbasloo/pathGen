var map = new OSMBuildings({
  container: 'map',
  position:{latitude: 50.949650, longitude: 6.933180},
  zoom: 16,
  minZoom: 10,
  maxZoom: 20,
  tilt: 40,
  rotation: 300,
  effects: ['shadows'],
  attribution: '© Data <a href="https://openstreetmap.org/copyright/">OpenStreetMap</a> © Map <a href="https://mapbox.com/">Mapbox</a> © 3D <a href="https://osmbuildings.org/copyright/">OSM Buildings</a>'
});

map.addMapTiles('https://{s}.tiles.mapbox.com/v3/osmbuildings.kbpalbpk/{z}/{x}/{y}.png');
map.addGeoJSONTiles('https://{s}.data.osmbuildings.org/0.2/dixw8kmb/tile/{z}/{x}/{y}.json');
map.addMarker({ latitude: 50.949650, longitude: 6.933180, altitude:30}, {color: 'blue'});


