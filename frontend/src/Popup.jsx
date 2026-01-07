// Popup.jsx 상단에 import 추가
import React, { useState } from "react";
import TimeSlotPicker from "./TimeSlotPicker"; // 👈 추가
const BACKEND_URL = "http://127.0.0.1:8000";
export default function Popup({ isOpen, onClose, onNext, orderSummary }) {
  const [pickupType, setPickupType] = useState("onsite");
  const [reservationTime, setReservationTime] = useState(""); // 여기에 "2025-01-01 14:30" 같은 문자열이 담김

  if (!isOpen) return null;

  // Popup.jsx 내부

  const handleNextStep = async () => {
    // 1. 예약인데 시간 선택 안 한 경우
    if (pickupType === "reservation" && !reservationTime) {
      alert("예약 픽업 시간을 선택해주세요!");
      return;
    }

    // 2. 서버 검증 요청
    const checkTime = pickupType === "onsite" ? "now" : reservationTime;
    
    try {
        const res = await fetch(`${BACKEND_URL}/api/check_availability`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ time: checkTime })
        });
        const result = await res.json();

        // ✅ [CASE 1] 바로 가능함
        if (result.available) {
            onNext({
                type: pickupType,
                time: pickupType === "onsite" ? "now" : reservationTime,
            });
        } 
        // ❌ [CASE 2] 불가능함 (추천 시간 제안)
        else {
            // 추천 시간이 같이 왔는지 확인
            if (result.recommended_time) {
                const msg = `⚠️ 현재 대기 중인 주문이 있습니다.\n\n가장 빠른 [ ${result.rec_msg} ] 에 픽업하시겠습니까?`;
                
                // 사용자가 확인(OK)을 누르면 그 시간으로 예약 진행
                if (window.confirm(msg)) {
                    onNext({
                        type: 'onsite', // 타입은 현장이지만
                        time: result.recommended_time // 시간은 미래 시간으로 지정
                    });
                }
            } else {
                // 추천 시간이 없으면 그냥 에러 메시지
                alert(`예약 불가: ${result.message}`);
            }
        }

    } catch (err) {
        console.error(err);
        alert("서버 연결 오류로 가능 여부를 확인할 수 없습니다.");
    }
  };

  return (
    <div className="fixed inset-0 bg-black/60 flex items-center justify-center z-[9999] backdrop-blur-sm animate-in fade-in zoom-in duration-300">
      <div className="bg-white w-full max-w-md rounded-3xl p-6 shadow-2xl border-4 border-indigo-100 relative overflow-hidden flex flex-col max-h-[90vh]">
        
        {/* 헤더 */}
        <div className="text-center mb-4 shrink-0">
          <h2 className="text-2xl font-black text-slate-800">🛍️ 픽업 방법 선택</h2>
          <p className="text-gray-400 text-xs mt-1">어떻게 수령하시겠습니까?</p>
        </div>

        {/* 픽업 타입 선택 버튼들 */}
        <div className="grid grid-cols-2 gap-3 mb-4 shrink-0">
          <button
            onClick={() => setPickupType("onsite")}
            className={`p-3 rounded-2xl border-2 flex flex-col items-center gap-1 transition-all ${
              pickupType === "onsite"
                ? "border-indigo-500 bg-indigo-50 text-indigo-700 ring-2 ring-indigo-200"
                : "border-slate-100 bg-white text-slate-400 hover:border-indigo-200"
            }`}
          >
            <span className="text-2xl">🏃</span>
            <span className="font-bold text-sm">현장 픽업</span>
          </button>

          <button
            onClick={() => setPickupType("reservation")}
            className={`p-3 rounded-2xl border-2 flex flex-col items-center gap-1 transition-all ${
              pickupType === "reservation"
                ? "border-rose-500 bg-rose-50 text-rose-700 ring-2 ring-rose-200"
                : "border-slate-100 bg-white text-slate-400 hover:border-rose-200"
            }`}
          >
            <span className="text-2xl">📅</span>
            <span className="font-bold text-sm">예약 픽업</span>
          </button>
        </div>

        {/* --- 여기부터 내용이 바뀝니다 --- */}
        
        {/* 1. 현장 픽업일 때 안내 문구 */}
        {pickupType === "onsite" && (
            <div className="flex-1 flex flex-col items-center justify-center bg-indigo-50 rounded-2xl p-6 mb-4 animate-in fade-in">
                <div className="w-16 h-16 bg-white rounded-full flex items-center justify-center text-3xl mb-3 shadow-sm">🚀</div>
                <p className="font-bold text-indigo-900">지금 바로 준비할까요?</p>
                <p className="text-xs text-indigo-400 mt-1">대기시간: 약 15분 예상</p>
            </div>
        )}

        {/* 2. 예약 픽업일 때 시간 선택기(TimeSlotPicker) 등장 */}
        {pickupType === "reservation" && (
          <div className="flex-1 overflow-hidden flex flex-col mb-4 animate-in slide-in-from-bottom-4 fade-in">
            <label className="text-xs font-bold text-slate-500 mb-2 block">픽업 시간 선택 (24H)</label>
            {/* 👇 새로 만든 컴포넌트 사용 */}
            <TimeSlotPicker onSelect={(time) => setReservationTime(time)} />
          </div>
        )}

        {/* 하단 버튼 */}
        <div className="grid grid-cols-3 gap-3 shrink-0">
          <button
            onClick={onClose}
            className="col-span-1 py-3 bg-gray-100 hover:bg-gray-200 text-gray-500 font-bold rounded-xl transition"
          >
            취소
          </button>
          <button
            onClick={handleNextStep}
            className={`col-span-2 py-3 text-white font-bold rounded-xl shadow-lg transition transform active:scale-95 flex justify-center items-center gap-2 ${
                pickupType === "reservation" && !reservationTime 
                ? "bg-gray-300 cursor-not-allowed" // 시간 선택 안하면 회색
                : "bg-gradient-to-r from-blue-600 to-indigo-600 hover:from-blue-700 hover:to-indigo-700"
            }`}
            disabled={pickupType === "reservation" && !reservationTime}
          >
            <span>💳</span> 결제하기
          </button>
        </div>

      </div>
    </div>
  );
}