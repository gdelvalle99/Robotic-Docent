import React, { useState, useEffect, useRef } from "react";
import { Fab } from "@material-ui/core";
import { mapLink, floor_id } from "../links";
import { ExhibitModalPiece } from "./ExhibitModalPiece";
import Add from "@material-ui/icons/Add";
import PieceButton from "./PieceButton";
import PiecePreviewModal from "./PiecePreviewModal";

function useTraceUpdate(props) {
  const prev = useRef(props);
  useEffect(() => {
    const changedProps = Object.entries(props).reduce((ps, [k, v]) => {
      if (prev.current[k] !== v) {
        ps[k] = [prev.current[k], v];
      }
      return ps;
    }, {});
    // if (Object.keys(changedProps).length > 0) {
    //   console.log('Changed props:', changedProps);
    // }
    prev.current = props;
  });
}

export const Map = (props) => {
    
    const [image, setImage] = useState("");
    const [open, setOpen] = useState(false);
    const [openPiece, setOpenPiece] = useState(false);
    const [piece, setPiece] = useState(null);
    const [el, setEl] = useState(null);
    const [loc, setLoc] = useState([]);
    useTraceUpdate({image,open,openPiece,piece,el});
    
    const handleClose = () => {
        setOpen(false);
    };

    const handleMouseOver = (piece, event) => {
      //console.log("mouse over", piece)
      setPiece(piece);
      setEl(event.currentTarget);
    }

    const handleOpenClick = () => {
        setOpen(true);
    };

    const handlePieceOpen = () => {
        //console.log("ayo, we clicked")
        setOpenPiece(true);
    };

    const handlePieceClose = () => {
        setOpenPiece(false);
        setPiece(null);
    };

    const exitPiece = () => {
      if (!openPiece) setTimeout(() => setPiece(null), 50);
  };

    const getImage = () => {
        const formData = new FormData();
        formData.append("floor_id", floor_id); // just for testing

        const token = localStorage.getItem("auth_token") || "";
        fetch(mapLink, {
            method: "POST",
            headers: {
                Accept: "application/json, text/plain, */*",
                Authentification: token,
                credentials: true,
            },
            body: formData,
        })
            .then((response) => {
                return response.blob();
            })
            .then((blob) => {
                const link = URL.createObjectURL(blob);
                setImage(link);
            })
            .catch((e) => console.log(e));
    };
    useEffect(() => {
        getImage();
    }, []);

    const places_arr = [
        {
            title: "The Physical Impossibility of Death in the Mind of Someone Living",
            author: "Damien Hirst",
            description: "Conserved in formaldehyde, the work The Physical Impossibility of Death in the Mind of Someone Living by Damien Hirst is still today one of the most controversial pieces of contemporary art.",
            coordinates: [45, 50],
            img: "https://www.damienhirst.com/images/hirstimage/DHS76_771_0.jpg"
        },
        { 
            title: "(Untitled)", 
            author: "Keith Haring", 
            description: "From his beginnings as a graffiti artist in the New York subway, Keith Haring began his career with his immediately recognizable figures and patterns. One of his most commonly represented symbols is the heart.",
            coordinates: [25, 85],
            img: "https://imgc.artprintimages.com/img/print/untitled-c-1988_u-l-f4y1ut0.jpg?h=550&p=0&w=550&background=fbfbfb" 
        },
        
    ];

    const [places, setPlaces] = useState(places_arr)

    const [svg,setSvg] = useState(null);

    useEffect(()=>{
      let oop = document.getElementById('map')
      setSvg(oop)
    }, [])

    function alert_coords(evt) {
        let pt = svg.createSVGPoint()
        pt.x = evt.clientX;
        pt.y = evt.clientY;
    
        // The cursor point, translated into svg coordinates
        let cursorpt =  pt.matrixTransform(svg.getScreenCTM().inverse());
        return [cursorpt.x, cursorpt.y]
    }

    const handleMouseMove = function(event) {
      if(!editLoc) return;
      let ans = alert_coords(event);
      ans[0] -= 3;
      ans[1] -= 6;
      setLoc(ans);
      handleOpenClick();
      setEditLoc(false);
    }

    const [editLoc, setEditLoc] = useState(false);

    const handleLocation = () => {
      setEditLoc(true);
      handleClose();
    }

    const addToPlaces = (place) => {
      setPlaces(prev=>[...prev, {...place, coordinates: loc}]);
      setLoc([]); // Since we have a spot, we remove the dot
    }

    return (
        <div className="map-portion">
            <div className="img-overlay-wrap" onClick={handleMouseMove}>
                {image && <img src={image} className="map-image" alt="iwi" />}
                <svg id="map" className="exhibits" viewBox="0 0 200 250">
                    <defs>
                        <symbol id="pin" className="millet" viewBox="0 0 24 24">
                            <path
                                id="thePath"
                                d="M12 2c-3.87 0-7 3.13-7 7 0 5.25 7 13 7 13s7-7.75 7-13c0-3.87-3.13-7-7-7zm0 9.5c-1.38 0-2.5-1.12-2.5-2.5s1.12-2.5 2.5-2.5 2.5 1.12 2.5 2.5-1.12 2.5-2.5 2.5z"
                            ></path>
                            <path d="M0 0h24v24h-24z" fill="none"></path>
                        </symbol>
                        <symbol id="red_pin" className="millet" viewBox="0 0 24 24">
                            <path
                                id="thePath"
                                d="M12 2c-3.87 0-7 3.13-7 7 0 5.25 7 13 7 13s7-7.75 7-13c0-3.87-3.13-7-7-7zm0 9.5c-1.38 0-2.5-1.12-2.5-2.5s1.12-2.5 2.5-2.5 2.5 1.12 2.5 2.5-1.12 2.5-2.5 2.5z"
                                fill="red"
                            ></path>
                            <path d="M0 0h24v24h-24z" fill="none"></path>
                        </symbol>
                    </defs>

                    {/* {piece && <MapPopUp piece={piece} />} */}
                    <polygon
                        style={{ opacity: "20%", fill: "blue" }}
                        points="10,80.5 46,80.5 45.5,104 9.7,103"
                    />
                    <polygon
                        style={{ opacity: "20%", fill: "green" }}
                        points="30,42.5 63,44.5 62,62 30,61"
                    />
                    <polygon
                        style={{ opacity: "20%", fill: "yellow" }}
                        points="135.5,62.3 95.4,62.4 95.5,100.2 135.5,100.5"
                    />
                    {places.map((p) => (
                        <use
                            onClick={handlePieceOpen}
                            // onMouseOver={(e)=>handleMouseOver(p,e)}
                            onMouseEnter={(e)=>handleMouseOver(p,e)}
                            // onMouseOut={exitPiece}
                            xlinkHref="#pin"
                            x={p.coordinates[0]}
                            y={p.coordinates[1]}
                            width="6"
                            height="6"
                        />
                        // <Marker piece={p}/>
                    ))}
                    {/* <Marker/> */}
                    {loc.length === 2 &&                         <use
                            onClick={handlePieceOpen}
                            // onMouseOver={(e)=>handleMouseOver(p,e)}
                            // onMouseEnter={(e)=>handleMouseOver(p,e)}
                            // onMouseOut={exitPiece}
                            xlinkHref="#red_pin"
                            x={loc[0]}
                            y={loc[1]}
                            width="6"
                            height="6"
                        />}
                    {piece && <PiecePreviewModal piece={piece} openPiece={handlePieceOpen} anchorEl={el} setAnchor={setEl}/>}

                </svg>
            </div>
            
            {/* <button type="button" onClick={()=>setModalOpen(true)}>
            Open Modal
          </button> */}

            <PieceButton open={open} coordinates={loc} setLoc={setLoc} handleLocation={handleLocation} addToPlaces={addToPlaces} handleOpenClick={handleOpenClick} handleClose={handleClose}/>
            <ExhibitModalPiece
                open={openPiece}
                handleClose={handlePieceClose}
                piece={piece}
            />
        </div>
    );
};

export default Map;
