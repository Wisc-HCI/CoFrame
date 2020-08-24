const INITIAL_XML = '<xml xmlns="http://www.w3.org/1999/xhtml">\n'
    + '</xml>';

const INITIAL_TOOLBOX_XML = '<xml xmlns="http://www.w3.org/1999/xhtml" id="toolbox" style="display: none;">\n'
    + '  <category name="Tasks" colour="#5C81A6">\n'
    + '  </category>\n'
    + '  <category name="Primitives" colour="#5CA65C">\n'
    + '  </category>\n'
    + '  <category name="Locations" colour="#A6745C" custom="LOCATIONS">\n'
    + '    <button text="Create Location" callbackKey="createLocationPressed"></button>\n'
    + '  </category>\n'
    + '  <category name="Machines" colour="#A6745C" custom="MACHINES">\n'
    + '    <button text="Create Machine" callbackKey="createMachinePressed"></button>\n'
    + '  </category>\n'
    + '  <category name="Objects" colour="#A6745C" custom="OBJECTS">\n'
    + '    <button text="Create Object" callbackKey="createObjectPressed"></button>\n'
    + '  </category>\n'
    + '  <category name="Functions" colour="#9A5CA6" custom="PROCEDURE"></category>\n'
    + '</xml>';

const INITIAL_TOOLBOX_CATEGORIES = [

];

const ConfigFiles = {
  INITIAL_XML,
  INITIAL_TOOLBOX_XML,
  INITIAL_TOOLBOX_CATEGORIES,
};

export default ConfigFiles;