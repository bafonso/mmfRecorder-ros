
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

  // My Camera Controller
//  var shutterSpeedParam = new ROSLIB.Param({
//    ros : ros,
//    name : '/camera/camera_nodelet/shutter_speed'
//    name: 'test'
// });

  var shutterSpeedParam = ros.Param({
      ros:ros,
      name: '/camera/camera_nodelet/shutter_speed'
  });

  $('#slider-shutter-speed').slider({
    orientation: 'horizontal',
    range: 'min',
    min: 0,
    max: 1,
    value: 0.03,
    step: 0.001,
    slide: function(event, ui) {
      $('#shutter-speed').val(ui.value);
      shutterSpeedParam.set(ui.value);
    }
  });
  $('#shutter-speed').val($('#slider-shutter-speed').slider('value'));


// Using actions
  var recordClient = new ROSLIB.ActionClient({
    ros : ros,
    serverName : '/camera/mmf_recorder',
    actionName : '/mmf_writer/RecordAction'
  });



  var recordGoal;
  var secondsToRecord = 600;
  var fps_input = 30;
  var keyframe = 120;
  var filename = 'defaultMMFFile';
  var basePath = '/home/bruno/recordings';
  var tracker = 't10';
  var gal4_line = 'attP2'
  var effector = 'tntE'
  var stimuli_description = 'r_150112s5_u8_256i_div_6_d_900_6s60x2s900s#n#n#n';
  stimuli_description = 'n#n#n#n';
  var finalPath ;
  var finalFileName;
  // we set some defaults
  $('#filename').val(filename);
  $('#basePath').val(basePath);
  $('#timeToRecord').val(secondsToRecord);
  $('#fps').val(fps_input);
  $('#keyframe').val(keyframe);
  $('#tracker').val(tracker);
  $('#gal4_line').val(gal4_line);
  $('#effector').val(effector);
  $('#stimuli_description').val(stimuli_description);
  $('#data_directory').val('please make some changes');

function addZ(n){return n<10? '0'+n:''+n;}

function updateSettings() {
  var d = new Date();
  console.log(d.getDate());

  finalPath = basePath + '/' + tracker + '/' + stimuli_description + '/' + gal4_line + '(' + effector + ')' + '/' +
  d.getFullYear() + '/' + ("0" + (d.getMonth() + 1)).slice(-2) + '/' + addZ(d.getDate()) + '/' + addZ(d.getHours())
  + addZ(d.getMinutes()) ;

//GMR_SS04330@Chrimson@t10@r_150112s5_u8_256i_div_6_d_900_6s60x2s900s#n#n#n@30                                                                             
  finalFileName = gal4_line + '@' + effector + '@' + tracker + '@' + stimuli_description + '@30' ; //+ '.mmf' ;
  $('#file_name').val(finalFileName);

  $('#data_directory').val(finalPath);
  console.log (finalPath);
  secondsToRecord = $('#timeToRecord').val() ;

}

updateSettings();

$('#tracker').change( function() {
  //console.log($('#tracker').val());
  tracker = $('#tracker').val();
  updateSettings();
});

$('#basePath').on('input', function() {
  basePath = $('#basePath').val();
  updateSettings();
});


$('#effector').on('input', function() {
  effector = $('#effector').val();
  updateSettings();
});


$('#stimuli_description').on('input', function() {
  stimuli_description = $('#stimuli_description').val();
  updateSettings();
});

// C:\data\ChrimsonNoise\t10\r_150112s5_u8_256i_div_6_d_900_6s60x2s900s#n#n#n\GMR_SS04330(Chrimson)\2016\02\09\1518                                                                                                                 
  $('#timeToRecord').on('input', function() {
    updateSettings();
    console.log("changed it to " + $(this).val());
  });

  $('#gal4_line').on('input', function() {
    gal4_line = $('#gal4_line').val();
    updateSettings();
    console.log("changed it to " + $(this).val());
  });

    $('#mmf-controller-start').bind('click', function() {
      console.log('We send goal');
      console.log (finalPath);
      console.log (finalFileName);

      recordGoal = new ROSLIB.Goal({
        actionClient : recordClient,
        goalMessage : {
          recorder_settings: {
              timeToRecord: parseInt(secondsToRecord),
              fileName: finalFileName,
              path: finalPath,
              keyframe_interval: keyframe,
              fps: fps_input
          }
        }
    });

      recordGoal.on('feedback', function(feedback) {
//           console.log('Feedback: ' + feedback.recording_feedback.buffer);
           $('#recordingBuffer').val(feedback.recording_feedback.buffer) ;
           $('#recordingElapsedTime').val(feedback.recording_feedback.elapsed_recording.toFixed(2)) ;
           $('#lostFrames').val(feedback.recording_feedback.lost_frames) ;
//           console.log(feedback.recording_feedback.lost_frames);
        $('#recordingStatus').val('Recording!');
        });

        recordGoal.on('result', function(feedback) {
          console.log("We reach the result!");
          $('#recordingStatus').val('Success!');
        });

      recordGoal.send();
    });

    $('#mmf-controller-stop').bind('click', function() {
      console.log('We send preempt');
      recordGoal.cancel();
    });






















  // // My MMF Controller
  // var startRecordingController = new ROSLIB.Topic({
  //   ros : ros,
  //   name : '/camera/mmf_writer/start',
  //   messageType : 'std_msgs/Empty'
  // });
  //
  // var stopRecordingController = new ROSLIB.Topic({
  //   ros : ros,
  //   name : '/camera/mmf_writer/stop',
  //   messageType : 'std_msgs/Empty'
  // });

  // $('#mmf-controller-start').bind('click', function() {
  //   startRecordingController.publish(emptyMsg);
  //   console.log('mmf-controller-start');
  // });
  //
  // $('#mmf-controller-stop').bind('click', function() {
  //   stopRecordingController.publish(emptyMsg);
  //   console.log('mmf-controller-stop');
  // });
  //

// Need to change how long to record, etc

    // var secondsToRecordParam = ros.Param({
    //     ros:ros,
    //     name: '/camera/mmf_writer/secs_to_record'
    // });
// secondsToRecord.set(77, function(value) {
//   console.log(value.name);
//   console.log('set changed it to: ' + value);
// });



// secondsToRecord.set(function(done) {
//   secondsToRecord.set(20.0);
//   //setTimeout(done, 500);
// });


// secondsToRecordParam.set(2, function(value) {
//   console.log("Fuck");
//   setTimeout(value,500);
// //   secondsToRecord.set($(this).val());
// //   console.log("changed it to " + $(this).val());
// });
//
//
//
// var secondsToRecord = new ROSLIB.Param({
//     ros : ros,
//     name : '/camera/mmf_writer/secs_to_record'
//   });
//
// //  secondsToRecord.set(2, function(value) {
// //    console.log("We enter sounds");
// // //   setTimeout(value,500);
// // //   secondsToRecord.set($(this).val());
// // //   console.log("changed it to " + $(this).val());
// // });
//
// $('#timeToRecord').on('input', function() {
//   secondsToRecord.set(parseInt($(this).val()));
// //  secondsToRecord.set(1.2);
//
//   console.log("changed it to " + $(this).val());
//   });
//
//
//
// secondsToRecord.set(25.0);
//
// secondsToRecord.get(function(value) {
//   console.log("Yea");
//   console.log('MAX VAL: ' + value);
// });




// Monitoring the recording status
var recordingStatus = new ROSLIB.Topic({
  ros : ros,
  name : '/camera/recordingStatus',
  messageType : 'mmf_writer/RecordingStatus'
});


recordingStatus.subscribe(function(message) {
  var buffer_count_string = 'buffer # :' + message.buffer;
//  console.log(message.elapsedRecording/1000);
  $('#recordingBuffer').val(message.buffer) ;
  $('#recordingElapsedTime').val(message.elapsedRecording/1000) ;

});


$('#slider-crop').slider({
  orientation: 'horizontal',
  range: 'min',
  min: 1,
  max: 20,
  value: 0,
  step: 0.1,
  slide: function(event, ui) {
    $('#crop_value').val(ui.value);
    widthValue = ui.value * 600 ;
    $( '#camera-img' ).css( "width", widthValue + 'px');
    $( '#camera-img' ).css( "height", widthValue + 'px');
//    $('#stream-camera-image-raw').css()
    //currentMsg.current = ui.value;
  }
});

$('#pan-horizontal-slider').slider({
  orientation: 'horizontal',
  range: 'min',
  min: -800,
  max: 800,
  value: 0,
  step: 1,
  slide: function(event, ui) {
    $('#pan-horizontal-slider_value').val(ui.value);
    $( '#camera-img' ).css( "margin-left", ui.value);
  }
});

$('#pan-vertical-slider').slider({
  orientation: 'horizontal',
  range: 'min',
  min: -800,
  max: 800,
  value: 0,
  step: 1,
  slide: function(event, ui) {
    $('#pan-vertical-slider_value').val(ui.value);
    $( '#camera-img' ).css( "margin-top", ui.value);
  }
});



//text(message.buffer);
//$('#recordingBuffer').val('algo') ;


// var currentMsg = new ROSLIB.Message({
//   channel : 1,
//   current : 0
// });
//
// var cmdCurrent = new ROSLIB.Topic({
//   ros : ros,
//   name : '/mightex_controller_node/cmd_current',
//   messageType : 'mightex_controller/CmdCurrent'
// });
//
// $('#slider-current').slider({
//   orientation: 'horizontal',
//   range: 'min',
//   min: 0,
//   max: 1000,
//   value: 0,
//   step: 100,
//   slide: function(event, ui) {
//     $('#current').val(ui.value);
//     currentMsg.current = ui.value;
//   }
// });
// $('#current').val($('#slider-current').slider('value'));
//






















//$('#duration').val($('#slider-duration').slider('value'));

  //
  //
  //
  //
  //
  // var startStageController = new ROSLIB.Topic({
  //   ros : ros,
  //   name : '/camera/lavis_stage_controller_node/start',
  //   messageType : 'std_msgs/Empty'
  // });
  //
  // var stopStageController = new ROSLIB.Topic({
  //   ros : ros,
  //   name : '/camera/lavis_stage_controller_node/stop',
  //   messageType : 'std_msgs/Empty'
  // });
  //
  // var saveBackgroundImage = new ROSLIB.Topic({
  //   ros : ros,
  //   name : '/camera/blob_tracker/save_background_image',
  //   messageType : 'std_msgs/Empty'
  // });
  //
  // var startDataWriter = new ROSLIB.Topic({
  //   ros : ros,
  //   messageType : 'std_msgs/Empty'
  // });
  //
  // var stopDataWriter = new ROSLIB.Topic({
  //   ros : ros,
  //   name : '/camera/larvae_data_writer/stop',
  //   messageType : 'std_msgs/Empty'
  // });
  //
  // var startStimuliController = new ROSLIB.Topic({
  //   ros : ros,
  //   name : '/camera/lavis_stimuli_controller_node/start',
  //   messageType : 'std_msgs/Empty'
  // });
  //
  // var stopStimuliController = new ROSLIB.Topic({
  //   ros : ros,
  //   name : '/camera/lavis_stimuli_controller_node/stop',
  //   messageType : 'std_msgs/Empty'
  // });
  //
  // var behaviors = new ROSLIB.Topic({
  //   ros : ros,
  //   name : '/camera/behaviors',
  //   messageType : 'larvae_behavior_classifier/Behaviors'
  // });
  //
  // var moveAbsolutePercentClient = new ROSLIB.ActionClient({
  //   ros : ros,
  //   serverName : '/zaber_stage_node/move_absolute_percent',
  //   actionName : '/zaber_stage/MoveAction'
  // });
  //
  // var moveGoal = new ROSLIB.Goal({
  //   actionClient : moveAbsolutePercentClient,
  //   goalMessage : {
  //     pose : {
  //       position : {
  //         x : 50,
  //         y : 50,
  //         z : 0
  //       },
  //       orientation : {
  //         x : 0,
  //         y : 0,
  //         z : 0,
  //         w : 0
  //       }
  //     }
  //   }
  // });
  //
  // moveGoal.on('result', function(result) {
  //   console.log('Final Move Position: ' + result.pose);
  // });
  //
  // $('#slider-stage-vertical').slider({
  //   orientation: 'vertical',
  //   range: 'min',
  //   min: 0,
  //   max: 100,
  //   value: 50,
  //   slide: function(event, ui) {
  //     $('#stage-vertical-percent').val(100 - ui.value);
  //     moveGoal.goalMessage.goal.pose.position.x = 100 - ui.value;
  //   }
  // });
  // $('#stage-vertical-percent').val($('#slider-stage-vertical').slider('value'));
  //
  // $('#slider-stage-horizontal').slider({
  //   orientation: 'horizontal',
  //   range: 'min',
  //   min: 0,
  //   max: 100,
  //   value: 50,
  //   slide: function(event, ui) {
  //     $('#stage-horizontal-percent').val(ui.value);
  //     moveGoal.goalMessage.goal.pose.position.y = ui.value;
  //     console.log(moveGoal);
  //   }
  // });
  // $('#stage-horizontal-percent').val($('#slider-stage-horizontal').slider('value'));
  //
  // var currentMsg = new ROSLIB.Message({
  //   channel : 1,
  //   current : 0
  // });
  //
  // var cmdCurrent = new ROSLIB.Topic({
  //   ros : ros,
  //   name : '/mightex_controller_node/cmd_current',
  //   messageType : 'mightex_controller/CmdCurrent'
  // });
  //
  // $('#slider-current').slider({
  //   orientation: 'horizontal',
  //   range: 'min',
  //   min: 0,
  //   max: 1000,
  //   value: 0,
  //   step: 100,
  //   slide: function(event, ui) {
  //     $('#current').val(ui.value);
  //     currentMsg.current = ui.value;
  //   }
  // });
  // $('#current').val($('#slider-current').slider('value'));
  //
  // var toneMsg = new ROSLIB.Message({
  //   frequency : 3000,
  //   duration : 500
  // });
  //
  // var playTone = new ROSLIB.Topic({
  //   ros : ros,
  //   name : '/pyaudio_controller_node/play_tone',
  //   messageType : 'pyaudio_controller/Tone'
  // });
  //
  // $('#slider-frequency').slider({
  //   orientation: 'horizontal',
  //   range: 'min',
  //   min: 1000,
  //   max: 15000,
  //   value: 3000,
  //   step: 1000,
  //   slide: function(event, ui) {
  //     $('#frequency').val(ui.value);
  //     toneMsg.frequency = ui.value;
  //   }
  // });
  // $('#frequency').val($('#slider-frequency').slider('value'));
  //
  // $('#slider-duration').slider({
  //   orientation: 'horizontal',
  //   range: 'min',
  //   min: 100,
  //   max: 2000,
  //   value: 500,
  //   step: 100,
  //   slide: function(event, ui) {
  //     $('#duration').val(ui.value);
  //     toneMsg.duration = ui.value;
  //   }
  // });
  // $('#duration').val($('#slider-duration').slider('value'));
  //
  // behaviors.subscribe(function(message) {
  //   // console.log('Received message on ' + behaviors.name + ': ' + message.behaviors[0].behavior);
  //   var behaviors_string = '';
  //   var behavior_count = message.behaviors.length;
  //   if (behavior_count > 1) {
  //     behaviors_string = 'behaviors: ';
  //   } else {
  //     behaviors_string = 'behavior: ';
  //   }
  //   for (var index=0;index<behavior_count;index++) {
  //     if (index > 0) {
  //       behaviors_string += ', ';
  //     }
  //     if (behavior_count > 1) {
  //       behaviors_string += index + ':';
  //     }
  //     behaviors_string += message.behaviors[index].behavior;
  //   }
  //   $('#behaviors').text(behaviors_string);
  //   // console.log(behaviors_string);
  // });
  //
  // $('#stage-move-absolute-percent').bind('click', function() {
  //   moveGoal.send();
  //   console.log('stage-move-absolute-percent');
  //   console.log(moveGoal);
  // });
  //
  //
  // $('#stage-controller-start').bind('click', function() {
  //   startStageController.publish(emptyMsg);
  //   console.log('stage-controller-start');
  // });
  //
  // $('#stage-controller-stop').bind('click', function() {
  //   stopStageController.publish(emptyMsg);
  //   console.log('stage-controller-stop');
  // });
  //
  // $('#save-background-image').bind('click', function() {
  //   saveBackgroundImage.publish(emptyMsg);
  //   console.log('save-background-image');
  // });
  //
  // $('#data-writer-start').bind('click', function() {
  //   startDataWriter.publish(emptyMsg);
  //   console.log('data-writer-start');
  // });
  //
  // $('#data-writer-stop').bind('click', function() {
  //   stopDataWriter.publish(emptyMsg);
  //   console.log('data-writer-stop');
  // });
  //
  // $('#stimuli-controller-start').bind('click', function() {
  //   startStimuliController.publish(emptyMsg);
  //   console.log('stimuli-controller-start');
  // });
  //
  // $('#stimuli-controller-stop').bind('click', function() {
  //   stopStimuliController.publish(emptyMsg);
  //   console.log('stimuli-controller-stop');
  // });
  //
  // $('#current-channel-1').bind('click', function() {
  //   currentMsg.channel = 1;
  //   cmdCurrent.publish(currentMsg);
  //   console.log('current-channel-1');
  // });
  //
  // $('#current-channel-2').bind('click', function() {
  //   currentMsg.channel = 2;
  //   cmdCurrent.publish(currentMsg);
  //   console.log('current-channel-2');
  // });
  //
  // $('#current-channel-3').bind('click', function() {
  //   currentMsg.channel = 3;
  //   cmdCurrent.publish(currentMsg);
  //   console.log('current-channel-3');
  // });
  //
  // $('#current-channel-4').bind('click', function() {
  //   currentMsg.channel = 4;
  //   cmdCurrent.publish(currentMsg);
  //   console.log('current-channel-4');
  // });
  //
  // $('#play-tone').bind('click', function() {
  //   playTone.publish(toneMsg);
  //   console.log('play-tone');
  // });

});
