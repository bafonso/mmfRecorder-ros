$(function() {
  $(':button').button();
  var tabs = $('#tabs').tabs();
  $(window).resize(function() {
    tabs.tabs('refresh');
  });

  // Connecting to ROS
  // -----------------
  var ros = new ROSLIB.Ros();

  // If there is an error on the backend, an 'error' emit will be emitted.
  ros.on('error', function(error) {
    $('#connecting').hide();
    $('#connected').hide();
    $('#closed').hide();
    $('#error').show();
    console.log(error);
  });

  // Find out exactly when we made a connection.
  ros.on('connection', function() {
    console.log('Connection made!');
    $('#connecting').hide();
    $('#error').hide();
    $('#closed').hide();
    $('#connected').show();
  });

  ros.on('close', function() {
    console.log('Connection closed.');
    $('#connecting').hide();
    $('#connected').hide();
    $('#closed').show();
  });

  // Create a connection to the rosbridge WebSocket server.
  ros.connect('ws://localhost:9090');

  var emptyMsg = new ROSLIB.Message({
  });

  var startStageController = new ROSLIB.Topic({
    ros : ros,
    name : '/camera/lavis_stage_controller_node/start',
    messageType : 'std_msgs/Empty'
  });

  var stopStageController = new ROSLIB.Topic({
    ros : ros,
    name : '/camera/lavis_stage_controller_node/stop',
    messageType : 'std_msgs/Empty'
  });

  var saveBackgroundImage = new ROSLIB.Topic({
    ros : ros,
    name : '/camera/blob_tracker/save_background_image',
    messageType : 'std_msgs/Empty'
  });

  var startDataWriter = new ROSLIB.Topic({
    ros : ros,
    name : '/camera/larvae_data_writer/start',
    messageType : 'std_msgs/Empty'
  });

  var stopDataWriter = new ROSLIB.Topic({
    ros : ros,
    name : '/camera/larvae_data_writer/stop',
    messageType : 'std_msgs/Empty'
  });

  var startStimuliController = new ROSLIB.Topic({
    ros : ros,
    name : '/camera/lavis_stimuli_controller_node/start',
    messageType : 'std_msgs/Empty'
  });

  var stopStimuliController = new ROSLIB.Topic({
    ros : ros,
    name : '/camera/lavis_stimuli_controller_node/stop',
    messageType : 'std_msgs/Empty'
  });

  var behaviors = new ROSLIB.Topic({
    ros : ros,
    name : '/camera/behaviors',
    messageType : 'larvae_behavior_classifier/Behaviors'
  });

  var moveAbsolutePercentClient = new ROSLIB.ActionClient({
    ros : ros,
    serverName : '/zaber_stage_node/move_absolute_percent',
    actionName : '/zaber_stage/MoveAction'
  });

  var moveGoal = new ROSLIB.Goal({
    actionClient : moveAbsolutePercentClient,
    goalMessage : {
      pose : {
        position : {
          x : 50,
          y : 50,
          z : 0
        },
        orientation : {
          x : 0,
          y : 0,
          z : 0,
          w : 0
        }
      }
    }
  });

  moveGoal.on('result', function(result) {
    console.log('Final Move Position: ' + result.pose);
  });

  $('#slider-stage-vertical').slider({
    orientation: 'vertical',
    range: 'min',
    min: 0,
    max: 100,
    value: 50,
    slide: function(event, ui) {
      $('#stage-vertical-percent').val(100 - ui.value);
      moveGoal.goalMessage.goal.pose.position.x = 100 - ui.value;
    }
  });
  $('#stage-vertical-percent').val($('#slider-stage-vertical').slider('value'));

  $('#slider-stage-horizontal').slider({
    orientation: 'horizontal',
    range: 'min',
    min: 0,
    max: 100,
    value: 50,
    slide: function(event, ui) {
      $('#stage-horizontal-percent').val(ui.value);
      moveGoal.goalMessage.goal.pose.position.y = ui.value;
      console.log(moveGoal);
    }
  });
  $('#stage-horizontal-percent').val($('#slider-stage-horizontal').slider('value'));

  var currentMsg = new ROSLIB.Message({
    channel : 1,
    current : 0
  });

  var cmdCurrent = new ROSLIB.Topic({
    ros : ros,
    name : '/mightex_controller_node/cmd_current',
    messageType : 'mightex_controller/CmdCurrent'
  });

  $('#slider-current').slider({
    orientation: 'horizontal',
    range: 'min',
    min: 0,
    max: 1000,
    value: 0,
    step: 100,
    slide: function(event, ui) {
      $('#current').val(ui.value);
      currentMsg.current = ui.value;
    }
  });
  $('#current').val($('#slider-current').slider('value'));

  var toneMsg = new ROSLIB.Message({
    frequency : 3000,
    duration : 500
  });

  var playTone = new ROSLIB.Topic({
    ros : ros,
    name : '/pyaudio_controller_node/play_tone',
    messageType : 'pyaudio_controller/Tone'
  });

  $('#slider-frequency').slider({
    orientation: 'horizontal',
    range: 'min',
    min: 1000,
    max: 15000,
    value: 3000,
    step: 1000,
    slide: function(event, ui) {
      $('#frequency').val(ui.value);
      toneMsg.frequency = ui.value;
    }
  });
  $('#frequency').val($('#slider-frequency').slider('value'));

  $('#slider-duration').slider({
    orientation: 'horizontal',
    range: 'min',
    min: 100,
    max: 2000,
    value: 500,
    step: 100,
    slide: function(event, ui) {
      $('#duration').val(ui.value);
      toneMsg.duration = ui.value;
    }
  });
  $('#duration').val($('#slider-duration').slider('value'));

  behaviors.subscribe(function(message) {
    // console.log('Received message on ' + behaviors.name + ': ' + message.behaviors[0].behavior);
    var behaviors_string = '';
    var behavior_count = message.behaviors.length;
    if (behavior_count > 1) {
      behaviors_string = 'behaviors: ';
    } else {
      behaviors_string = 'behavior: ';
    }
    for (var index=0;index<behavior_count;index++) {
      if (index > 0) {
        behaviors_string += ', ';
      }
      if (behavior_count > 1) {
        behaviors_string += index + ':';
      }
      behaviors_string += message.behaviors[index].behavior;
    }
    $('#behaviors').text(behaviors_string);
    // console.log(behaviors_string);
  });

  $('#stage-move-absolute-percent').bind('click', function() {
    moveGoal.send();
    console.log('stage-move-absolute-percent');
    console.log(moveGoal);
  });

  $('#stage-controller-start').bind('click', function() {
    startStageController.publish(emptyMsg);
    console.log('stage-controller-start');
  });

  $('#stage-controller-stop').bind('click', function() {
    stopStageController.publish(emptyMsg);
    console.log('stage-controller-stop');
  });

  $('#save-background-image').bind('click', function() {
    saveBackgroundImage.publish(emptyMsg);
    console.log('save-background-image');
  });

  $('#data-writer-start').bind('click', function() {
    startDataWriter.publish(emptyMsg);
    console.log('data-writer-start');
  });

  $('#data-writer-stop').bind('click', function() {
    stopDataWriter.publish(emptyMsg);
    console.log('data-writer-stop');
  });

  $('#stimuli-controller-start').bind('click', function() {
    startStimuliController.publish(emptyMsg);
    console.log('stimuli-controller-start');
  });

  $('#stimuli-controller-stop').bind('click', function() {
    stopStimuliController.publish(emptyMsg);
    console.log('stimuli-controller-stop');
  });

  $('#current-channel-1').bind('click', function() {
    currentMsg.channel = 1;
    cmdCurrent.publish(currentMsg);
    console.log('current-channel-1');
  });

  $('#current-channel-2').bind('click', function() {
    currentMsg.channel = 2;
    cmdCurrent.publish(currentMsg);
    console.log('current-channel-2');
  });

  $('#current-channel-3').bind('click', function() {
    currentMsg.channel = 3;
    cmdCurrent.publish(currentMsg);
    console.log('current-channel-3');
  });

  $('#current-channel-4').bind('click', function() {
    currentMsg.channel = 4;
    cmdCurrent.publish(currentMsg);
    console.log('current-channel-4');
  });

  $('#play-tone').bind('click', function() {
    playTone.publish(toneMsg);
    console.log('play-tone');
  });

});
