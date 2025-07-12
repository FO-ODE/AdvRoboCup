
"use strict";

let ASRService = require('./ASRService.js')
let GetSpeechDuration = require('./GetSpeechDuration.js')
let SoundLocalisationService = require('./SoundLocalisationService.js')
let recognizerService = require('./recognizerService.js')

module.exports = {
  ASRService: ASRService,
  GetSpeechDuration: GetSpeechDuration,
  SoundLocalisationService: SoundLocalisationService,
  recognizerService: recognizerService,
};
