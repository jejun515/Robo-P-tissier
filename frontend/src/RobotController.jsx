import React, { useState } from "react";
import { ref, update } from "firebase/database";

export default function RobotController({ db }) {
  // J1 ~ J6 관절 각도 상태 (기본값 0)
  const [joints, setJoints] = useState([0, 0, 0, 0, 0, 0]);

  // 슬라이더 변경 핸들러
  const handleJointChange = (index, value) => {
    const newJoints = [...joints];
    newJoints[index] = parseFloat(value);
    setJoints(newJoints);

    // Firebase에 실시간 업데이트 ('robot_control/joints')
    // (너무 잦은 업데이트 방지를 위해 실제로는 debounce 처리가 필요할 수 있음)
    update(ref(db, 'robot_control'), {
      joints: newJoints,
      updated_at: Date.now()
    });
  };

  // 버튼 명령 전송 핸들러
  const sendCommand = (cmd) => {
    update(ref(db, 'robot_control'), {
      command: cmd,
      timestamp: Date.now()
    });
    alert(`명령 전송: ${cmd}`);
  };

  return (
    <div className="bg-white p-5 rounded-2xl border border-gray-200 shadow-sm">
      <h3 className="text-sm font-bold text-gray-700 mb-4 flex items-center gap-2">
        🎮 로봇 수동 제어 (Manual Control)
      </h3>

      {/* 1. 기능 버튼 (안전 복구 / 홈) */}
      <div className="grid grid-cols-2 gap-3 mb-6">
        <button 
          onClick={() => sendCommand("recovery")}
          className="py-2 px-3 bg-amber-500 hover:bg-amber-600 text-white rounded-lg font-bold text-xs shadow-sm transition-colors flex items-center justify-center gap-1"
        >
          🚑 안전 복구 모드
        </button>
        <button 
          onClick={() => sendCommand("home")}
          className="py-2 px-3 bg-teal-600 hover:bg-teal-700 text-white rounded-lg font-bold text-xs shadow-sm transition-colors flex items-center justify-center gap-1"
        >
          🏠 Return Home
        </button>
      </div>

      {/* 2. 각 축 조절 슬라이더 (J1 ~ J6) */}
      <div className="space-y-4">
        {joints.map((angle, i) => (
          <div key={i} className="flex flex-col gap-1">
            <div className="flex justify-between text-xs font-bold text-gray-500">
              <span>Joint {i + 1}</span>
              <span className="text-blue-600">{angle}°</span>
            </div>
            <input
              type="range"
              min="-180"
              max="180"
              value={angle}
              onChange={(e) => handleJointChange(i, e.target.value)}
              className="w-full h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer accent-blue-600"
            />
          </div>
        ))}
      </div>
      
      <p className="text-[10px] text-gray-400 mt-4 text-center">
        * 슬라이더를 움직이면 로봇이 즉시 반응합니다. 주의하세요!
      </p>
    </div>
  );
}