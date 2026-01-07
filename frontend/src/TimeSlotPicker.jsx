// TimeSlotPicker.jsx
import React, { useState, useEffect } from "react";

// 백엔드 주소 (App.jsx와 동일하게 맞춤)
const BACKEND_URL = "http://127.0.0.1:8000";

export default function TimeSlotPicker({ onSelect }) {
  // 오늘 날짜 기준 1주일치 날짜 생성
  const [weekDates, setWeekDates] = useState([]);
  const [selectedDateStr, setSelectedDateStr] = useState(""); // 선택된 날짜 (YYYY-MM-DD)
  const [selectedTime, setSelectedTime] = useState(null);
  
  // 🚫 [NEW] 이미 예약된 시간 목록 상태
  const [reservedSlots, setReservedSlots] = useState([]);

  // 1. 초기 날짜 세팅
  useEffect(() => {
    const dates = [];
    const today = new Date();
    
    for (let i = 0; i < 7; i++) {
      const d = new Date(today);
      d.setDate(today.getDate() + i);
      dates.push(d);
    }
    setWeekDates(dates);
    
    // 기본값: 오늘 날짜 선택
    const todayStr = formatDate(dates[0]);
    setSelectedDateStr(todayStr);
  }, []);

  // 2. [NEW] 날짜가 변경될 때마다 서버에서 예약된 시간 가져오기
  useEffect(() => {
    if (selectedDateStr) {
      fetchReservedTimes(selectedDateStr);
    }
  }, [selectedDateStr]);

  const fetchReservedTimes = async (date) => {
    try {
      const response = await fetch(`${BACKEND_URL}/api/reserved_times`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ date: date }),
      });
      const data = await response.json();
      if (data.status === "success") {
        setReservedSlots(data.reserved_times); // 예: ["14:00", "14:15"]
      }
    } catch (error) {
      console.error("예약 정보 로딩 실패:", error);
    }
  };

  // 날짜 포맷 (YYYY-MM-DD)
  const formatDate = (date) => {
    const year = date.getFullYear();
    const month = String(date.getMonth() + 1).padStart(2, "0");
    const day = String(date.getDate()).padStart(2, "0");
    return `${year}-${month}-${day}`;
  };

  // 요일 포맷
  const getDayName = (date) => {
    const dayNames = ["일", "월", "화", "수", "목", "금", "토"];
    return dayNames[date.getDay()];
  };

  // 00:00 ~ 23:45 (15분 단위) 슬롯 생성
  const timeSlots = [];
  for (let h = 0; h < 24; h++) {
    for (let m = 0; m < 60; m += 15) {
      const hour = h.toString().padStart(2, "0");
      const min = m.toString().padStart(2, "0");
      timeSlots.push(`${hour}:${min}`);
    }
  }

  // 3. [UPGRADE] 시간 비활성화 로직 (과거 시간 + 예약된 시간)
  const isTimeDisabled = (timeStr) => {
    // 1) 이미 예약된 시간인지 확인
    if (reservedSlots.includes(timeStr)) return true;

    // 2) 과거 시간인지 확인 (오늘인 경우만)
    const todayStr = formatDate(new Date());
    if (selectedDateStr === todayStr) {
      const now = new Date();
      const currentHour = now.getHours();
      const currentMin = now.getMinutes();
      
      const [h, m] = timeStr.split(":").map(Number);
      
      // 현재 시간보다 이전이면 true
      if (h < currentHour) return true;
      if (h === currentHour && m < currentMin) return true;
    }
    
    return false;
  };

  const handleDateClick = (dateStr) => {
    setSelectedDateStr(dateStr);
    setSelectedTime(null); // 날짜 바뀌면 시간 초기화
  };

  const handleTimeClick = (time) => {
    if (isTimeDisabled(time)) return;
    setSelectedTime(time);
    onSelect(`${selectedDateStr} ${time}`);
  };

  return (
    <div className="w-full flex flex-col h-full overflow-hidden">
      
      {/* 1. 주간 달력 (가로 스크롤) */}
      <div className="flex gap-2 overflow-x-auto pb-2 mb-2 custom-scrollbar shrink-0">
        {weekDates.map((date, idx) => {
          const dateStr = formatDate(date);
          const isSelected = selectedDateStr === dateStr;
          const isToday = idx === 0;
          const dayName = getDayName(date);
          
          const isSun = date.getDay() === 0;
          const isSat = date.getDay() === 6;
          const dayColor = isSun ? "text-red-500" : isSat ? "text-blue-500" : "text-gray-500";

          return (
            <button
              key={dateStr}
              onClick={() => handleDateClick(dateStr)}
              className={`flex flex-col items-center justify-center min-w-[60px] p-2 rounded-xl border transition-all ${
                isSelected
                  ? "bg-slate-800 border-slate-800 text-white shadow-md ring-2 ring-offset-1 ring-slate-800"
                  : "bg-white border-slate-200 hover:bg-slate-50"
              }`}
            >
              <span className={`text-[10px] font-bold mb-1 ${isSelected ? "text-white" : dayColor}`}>
                {isToday ? "오늘" : dayName}
              </span>
              <span className={`text-sm font-black ${isSelected ? "text-white" : "text-slate-800"}`}>
                {date.getDate()}
              </span>
            </button>
          );
        })}
      </div>

      {/* 2. 시간 선택 그리드 */}
      <div className="flex-1 overflow-y-auto pr-1 custom-scrollbar">
        <div className="grid grid-cols-4 gap-2">
          {timeSlots.map((time) => {
            const disabled = isTimeDisabled(time); // 함수 사용
            const isReserved = reservedSlots.includes(time); // 예약돼서 막힌건지 확인용
            const isSelected = selectedTime === time;

            return (
              <button
                key={time}
                onClick={() => handleTimeClick(time)}
                disabled={disabled}
                className={`py-2 rounded-lg text-xs font-bold border transition-all relative ${
                  isSelected
                    ? "bg-rose-500 border-rose-500 text-white shadow-md scale-105"
                    : disabled
                    ? "bg-slate-100 border-transparent text-slate-300 cursor-not-allowed"
                    : "bg-white border-slate-200 text-slate-600 hover:border-rose-300 hover:bg-rose-50 hover:text-rose-600"
                }`}
              >
                {time}
                {/* 예약된 시간엔 작은 점 표시 */}
                {isReserved && !isSelected && (
                    <span className="absolute top-1 right-1 w-1.5 h-1.5 bg-red-400 rounded-full"></span>
                )}
              </button>
            );
          })}
        </div>
      </div>
      
      {/* 3. 하단 선택 정보 */}
      <div className="mt-3 text-center h-5 shrink-0">
        {selectedTime ? (
            <p className="text-xs font-bold text-slate-600 animate-in fade-in slide-in-from-bottom-1">
               📅 <span className="text-slate-800">{selectedDateStr}</span> <span className="text-rose-600 ml-1">{selectedTime}</span> 픽업
            </p>
        ) : (
            <p className="text-[10px] text-slate-400">원하시는 날짜와 시간을 선택해주세요.</p>
        )}
      </div>

      {/* 스크롤바 스타일 */}
      <style>{`
        .custom-scrollbar::-webkit-scrollbar {
          width: 4px;
          height: 4px;
        }
        .custom-scrollbar::-webkit-scrollbar-track {
          background: transparent;
        }
        .custom-scrollbar::-webkit-scrollbar-thumb {
          background: #cbd5e1;
          border-radius: 4px;
        }
        .custom-scrollbar::-webkit-scrollbar-thumb:hover {
          background: #94a3b8;
        }
      `}</style>
    </div>
  );
}