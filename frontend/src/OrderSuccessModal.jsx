// OrderSuccessModal.jsx
import React from "react";

export default function OrderSuccessModal({ isOpen, onClose, orderId, waitTime = 15 }) {
  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 bg-black/60 flex items-center justify-center z-[9999] backdrop-blur-sm animate-in fade-in zoom-in duration-300">
      <div className="bg-white w-full max-w-sm rounded-3xl p-8 shadow-2xl border-4 border-green-100 relative overflow-hidden text-center">
        
        {/* 배경 장식 (파티클 효과 느낌) */}
        <div className="absolute top-0 left-0 w-full h-2 bg-gradient-to-r from-green-400 via-blue-500 to-purple-500"></div>
        
        {/* 아이콘 */}
        <div className="w-20 h-20 bg-green-100 rounded-full flex items-center justify-center mx-auto mb-6 shadow-inner animate-bounce">
          <span className="text-4xl">✅</span>
        </div>

        {/* 메인 텍스트 */}
        <h2 className="text-2xl font-black text-slate-800 mb-2">주문이 접수되었습니다!</h2>
        <p className="text-gray-500 text-sm mb-6">
          로봇 셰프가 케이크를 만들기 시작합니다.<br/>
          잠시만 기다려주세요.
        </p>

        {/* 주문 정보 박스 */}
        <div className="bg-slate-50 border border-slate-200 rounded-2xl p-4 mb-6 space-y-3">
          <div className="flex justify-between items-center border-b border-slate-200 pb-2">
            <span className="text-xs font-bold text-slate-400 uppercase">Order No.</span>
            <span className="text-lg font-black text-blue-600">#{orderId}</span>
          </div>
          <div className="flex justify-between items-center">
            <span className="text-xs font-bold text-slate-400 uppercase">예상 대기시간</span>
            <div className="flex items-center gap-1">
              <span className="text-xl">⏳</span>
              <span className="text-lg font-bold text-slate-700">약 {waitTime}분</span>
            </div>
          </div>
        </div>

        {/* 확인 버튼 */}
        <button
          onClick={onClose}
          className="w-full py-4 bg-slate-900 hover:bg-slate-800 text-white font-bold rounded-2xl shadow-lg transition transform active:scale-95"
        >
          확인 (대기하기)
        </button>

      </div>
    </div>
  );
}