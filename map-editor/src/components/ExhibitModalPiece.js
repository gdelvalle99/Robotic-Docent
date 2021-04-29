import React, { useState, useEffect } from "react";
import { Button } from '@material-ui/core';
import { Dialog } from '@material-ui/core';
import { DialogTitle } from '@material-ui/core';
import { DialogContent } from '@material-ui/core';
import { DialogActions } from '@material-ui/core';
import { TextField } from '@material-ui/core';
import { Divider } from '@material-ui/core';
import { InputAdornment } from '@material-ui/core';
import DeleteIcon from '@material-ui/icons/Delete';

const convertSimpleISODate = function(dateString) {
    if (dateString) {
        let d = new Date(dateString);

        return d.toISOString().substring(0,10);
    }
    else {
        return null;
    }
}

export const ExhibitModalPiece = (props) => {
    const [piece, setPiece] = useState({});
    const [openDeleteAlert, setOpenDeleteAlert] = useState(false);

     //Handles alert for delete
     const handleOpenAlert = () => {
        if (piece.title != null && piece.title != "") {
            setOpenDeleteAlert(true);
        }
        else {
            props.handleDeletePiece(piece.id);
        }
    }

    const handleCloseAlert = () => {
        setOpenDeleteAlert(false);
    }

    const handleDeleteClick = () => {
        props.handleDeletePiece(piece.id);
        handleCloseAlert();
    }

    const setInitialPiece = () => {
        if (props.piece != null) {
            setPiece(props.piece);
            console.log(piece)
        }
    }

    useEffect(() => {
        setInitialPiece(); 
    }, [props]);

    return (
        <div className="exhibit-piece-item">
            <Dialog open={props.open} onClose={props.handleClose}>
                <DialogTitle>
                    <div className="exhibit-piece-modal-header">
                        <TextField
                            required={true} 
                            className="piece-item-title"
                            placeholder="Title" 
                            variant="outlined" 
                            name="title"
                            defaultValue={piece.title}
                            fullWidth
                            multiline
                        />
                        <TextField 
                            className="piece-item-author"
                            placeholder="Author" 
                            size={"small"}
                            variant="outlined" 
                            name="author"
                            defaultValue={piece.author}
                            fullWidth
                        />
                        <Divider variant="middle" />
                    </div>
                </DialogTitle>
                <DialogContent>
                    <div className="exhibit-piece-modal-content">
                        <TextField 
                            className="exhibit-piece-modal-description"
                            label="Description" 
                            variant="outlined" 
                            multiline 
                            rows={5}
                            name="description"
                            defaultValue={piece.description} 
                            fullWidth
                        />
                        <TextField 
                            className="exhibit-piece-modal-origin"
                            label="Origin" 
                            variant="outlined" 
                            name="origin"
                            defaultValue={piece.origin} 
                            fullWidth
                        />
                        <TextField 
                            className="exhibit-piece-modal-era"
                            label="Era" 
                            variant="outlined" 
                            name="era"
                            defaultValue={piece.era} 
                            fullWidth
                        />
                        <TextField 
                            className="exhibit-piece-modal-acquisition-date" 
                            label="Acquisition Date" 
                            variant="outlined" 
                            type="date" 
                            name="acquisition_date"
                            defaultValue={convertSimpleISODate(piece.acquisition_date)} 
                            InputLabelProps={{ shrink: true }}
                            fullWidth 
                        />
                        <div className="exhibit-piece-modal-dimension-container">
                            <TextField
                                label="Dimensions"
                                placeholder="Length"
                                name="0"
                                type="number"
                                variant="outlined"
                                defaultValue={piece.dimension && piece.dimension[0]} 
                                InputProps={{
                                    endAdornment: <InputAdornment position="end">in.</InputAdornment>
                                }}
                                InputLabelProps={{shrink:true}}
                            />
                            <p className="exhibit-piece-modal-dimension-dash">:</p>
                            <TextField
                                placeholder="Width"
                                name="1"
                                type="number"
                                variant="outlined"
                                defaultValue={piece.dimension && piece.dimension[1]} 
                                InputProps={{
                                    endAdornment: <InputAdornment position="end">in.</InputAdornment>
                                }}
                            />
                            <p className="exhibit-piece-modal-dimension-dash">:</p>
                            <TextField
                                placeholder="Height"
                                name="2"
                                type="number"
                                variant="outlined"
                                defaultValue={piece.dimension && piece.dimension[2]} 
                                InputProps={{
                                    endAdornment: <InputAdornment position="end">in.</InputAdornment>
                                }}
                            />
                        </div>
                        <div className="exhibit-piece-modal-coordinates-container">
                            <TextField
                                label="Coordinates"
                                placeholder="X"
                                name="0"
                                type="number"
                                variant="outlined"
                                defaultValue={piece.coordinates && piece.coordinates[0]} 
                                InputLabelProps={{shrink:true}}
                                InputProps={{
                                    startAdornment: <InputAdornment position="start">(</InputAdornment>
                                }}
                            />
                            <p className="exhibit-piece-modal-coordinates-comma">,</p>
                            <TextField
                                placeholder="Y"
                                name="1"
                                type="number"
                                variant="outlined"
                                defaultValue={piece.coordinates && piece.coordinates[1]} 
                                InputProps={{
                                    endAdornment: <InputAdornment position="end">)</InputAdornment>
                                }}
                            />
                        </div>
                    </div>
                </DialogContent>
                <DialogActions>
                    <div className="exhibit-piece-delete-button">
                        <Button onClick={handleOpenAlert} variant="outlined">
                            <DeleteIcon fontSize='small'/>
                        </Button>
                    </div>
                </DialogActions>
            </Dialog>
            <Dialog open={openDeleteAlert} onClose={handleCloseAlert}>
                <DialogTitle>Delete this exhibit?</DialogTitle>
                <DialogActions>
                    <div className="exhibit-piece-delete-alert-buttons">
                        <div className="exhibit-piece-delete-alert-button-delete">
                            <Button onClick={handleDeleteClick} variant="outlined">
                            Delete
                            </Button>
                        </div>
                        <div className="exhibit-piece-delete-alert-button-cancel">
                            <Button onClick={handleCloseAlert} variant="outlined">
                                Cancel
                            </Button>
                        </div>
                    </div>
                </DialogActions>
            </Dialog>
        </div>
    );
}