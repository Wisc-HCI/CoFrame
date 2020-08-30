const INITIAL_XML = '<xml xmlns="http://www.w3.org/1999/xhtml">\n' + '</xml>';

const INITIAL_TOOLBOX_XML =
  '<xml xmlns="http://www.w3.org/1999/xhtml" id="toolbox" style="display: none;">\n' +
  '  <category name="Locations" colour="#A6745C">\n' +
  '  </category>\n' +
  '  <category name="Waypoints" colour="#A6745C">\n' +
  '    <button text="Create Waypoint" callbackKey="createWaypointPressed"></button>\n' +
  '  </category>\n' +
  '</xml>';

const INITIAL_TOOLBOX_CATEGORIES = [];

const ConfigFiles = {
  INITIAL_XML,
  INITIAL_TOOLBOX_XML,
  INITIAL_TOOLBOX_CATEGORIES,
};

export default ConfigFiles;
