import { useState } from "react";
import PiecePreviewModal from "./PiecePreviewModal";

export default function Marker({ piece }) {
    console.log("rendered");
    const [el, setEl] = useState(null);
    if(!piece) return null;
    return (
        <svg>
            <defs>
                <symbol id="pin" className="millet" viewBox="0 0 24 24">
                    <path
                        id="thePath"
                        d="M12 2c-3.87 0-7 3.13-7 7 0 5.25 7 13 7 13s7-7.75 7-13c0-3.87-3.13-7-7-7zm0 9.5c-1.38 0-2.5-1.12-2.5-2.5s1.12-2.5 2.5-2.5 2.5 1.12 2.5 2.5-1.12 2.5-2.5 2.5z"
                    ></path>
                    <path d="M0 0h24v24h-24z" fill="none"></path>
                </symbol>
            </defs>

            <use
                // onClick={handlePieceOpen}
                // onMouseOver={(e)=>handleMouseOver(p,e)}
                onMouseEnter={(e) => setEl(e.currentTarget)}
                // onMouseOut={() => setEl(null)}
                xlinkHref="#pin"
                x={piece.coords[0]}
                y={piece.coords[1]}
                width="6"
                height="6"
            />
            <PiecePreviewModal piece={piece} anchorEl={el}/>
        </svg>
    );
}
