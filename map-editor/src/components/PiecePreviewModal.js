import { useState } from "react";
import { makeStyles } from "@material-ui/core/styles";
import Modal from "@material-ui/core/Modal";
import { Popover } from "@material-ui/core";

function getModalStyle() {
    const top = 20;
    const right = 5;

    return {
        top: `${top}%`,
        right: `${right}%`,
        // transform: `translate(-${top}%, -${left}%)`,
    };
}

const useStyles = makeStyles((theme) => ({
    paper: {
        position: "absolute",
        width: 400,
        backgroundColor: theme.palette.background.paper,
        border: "2px solid #000",
        boxShadow: theme.shadows[5],
        padding: theme.spacing(2, 4, 3),
    },
}));

const getDescription = (desc) => {
    const length = 80;
    if (desc == undefined || desc == null) return "";
    if (desc.length <= length) return desc;
    for (let i = length; i >= 0; i--) {
        let char = desc[i];
        if (char === " ") return desc.slice(0, i) + "...";
    }
    return desc.slice(0, length) + "...";
};

export default function PiecePreviewModal({ piece, openPiece, anchorEl, setAnchor }) {
    // console.log(piece)
    const classes = useStyles();
    // getModalStyle is not a pure function, we roll the style only on the first render
    const [modalStyle] = useState(getModalStyle);

    const body = (
        <div style={modalStyle} className={classes.paper}>
            <h2 id="simple-modal-title">{piece.title}</h2>
            <p id="simple-modal-description">
                {getDescription(piece.description)}
            </p>
            <div>
            <img
                src={piece.img}
                alt={"Photo of " + piece.title}
                style={{margin: 'auto'}}
                width={600/2}
                height={500/2}
            />
            </div>
        </div>
    );

    const open = Boolean(anchorEl);
    //console.log("open is", open)

    return (
        <div>
            <Modal
                open={open}
                aria-labelledby="simple-modal-title"
                aria-describedby="simple-modal-description"
                hideBackdrop
                // disableBackdropClick
                disableEnforceFocus
                disableAutoFocus
                onBackdropClick={()=>console.log("what")}
                onClick={()=>setAnchor(null)}
            >
                {body}
            </Modal>
            {/* <Popover
                id="mouse-over-popover"
                className={classes.popover}
                classes={{
                    paper: classes.paper,
                }}
                open={open}
                anchorEl={anchorEl}
                anchorOrigin={{
                    vertical: "bottom",
                    horizontal: "left",
                }}
                transformOrigin={{
                    vertical: "top",
                    horizontal: "left",
                }}
                // onClose={handlePopoverClose}
                disableRestoreFocus
            >
                {body}
            </Popover> */}
        </div>
    );
}
