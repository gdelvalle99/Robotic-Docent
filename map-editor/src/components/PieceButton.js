import React, { useState, useEffect } from "react";
import { Fab } from '@material-ui/core';
import { Button } from '@material-ui/core';
import { Dialog } from '@material-ui/core';
import { TextField } from '@material-ui/core';
import { DialogActions } from '@material-ui/core';
import { DialogContent } from '@material-ui/core';
import { DialogTitle } from '@material-ui/core';
import { Select } from '@material-ui/core';
import { MenuItem } from '@material-ui/core';
import { InputLabel } from '@material-ui/core';
import { InputAdornment } from '@material-ui/core';
import { floorLink } from "../links";
import axios from 'axios';
import Add from '@material-ui/icons/Add';

export const PieceButton = () => {
    const [open, setOpen] = useState(false);
    const [openDeleteAlert, setOpenDeleteAlert] = useState(false);
    const [exhibits, setExhibits] = useState([]);
    const [exhibitName, setExhibitName] = useState("");
    const [newDimension, setNewDimension] = useState([]);
    const [newCoordinates, setNewCoordinates] = useState([]);
    const [newPiece, setNewPiece] = useState({});

    //Handles alert for delete
    const handleOpenAlert = () => {
        if (newPiece.title != null && newPiece.title != "") {
            setOpenDeleteAlert(true);
        }
        else {
            handleCancel();
        }
    }

    const handleCloseAlert = () => {
        setOpenDeleteAlert(false);
    }

    const handleDeleteClick = () => {
        handleCancel();
        handleCloseAlert();
    }

    //Handle changes for text fields
    const handleDropdownChange = (event) => {
        setExhibitName(event.target.value);
    }

    const handleDimensionChange = (event) => {
        const newDim = newDimension;
        newDim[parseInt(event.target.name)] = event.target.value;
        setNewDimension(newDim);
        handleAllChanges(newPiece, {
            dimension: newDimension
        })
    }

    const handleCoordinateChange = (event) => {
        const newCoord = newCoordinates;
        newCoord[parseInt(event.target.name)] = event.target.value;
        setNewCoordinates(newCoord);
        handleAllChanges(newPiece, {
            coordinates: newCoordinates
        })
    }

    const handleChange = (event) => {
        handleAllChanges(newPiece, {
            [event.target.name]: event.target.value
        });
    }
    
    const handleAllChanges = (piece, diff) => {
        setNewPiece({...piece, ...diff});
    }

    //Handle save, cancel, open, and close
    const handleSave = () => {
        console.log(newPiece);
        handleClose();
    }

    const handleCancel = () => {
        setExhibitName("");
        setNewPiece({});
        handleClose();
    }

    const handleClose = () => {
        setOpen(false);
    }

    const handleOpenClick = () => {
        setOpen(true);
    }

    //Get initial exhibit list
    const getExhibits = () => {
        let r = axios.get(floorLink).
            then(function (response) {
                return response;
            }).then(item => {
                const e = item.data;
                setExhibits(e.exhibits);
            }).catch(e=>console.log(e))
    }

    useEffect(() => { 
        getExhibits(); 
    }, []);
    
    return (
        <div className="map-piece-button">
            <Fab aria-label="add" className="floating-piece-button" onClick={handleOpenClick}>
                <Add fontSize='large'/>
            </Fab>
            <Dialog open={open} onClose={handleCancel} aria-labelledby="form-dialog-title">
                <DialogTitle className="piece-modal-title-container">Add a New Piece</DialogTitle>
                <DialogContent>
                    <div className="piece-modal-content-container">
                        <TextField 
                            select
                            variant="outlined"
                            label="Exhibit"
                            value={exhibitName}
                            name="exhibit_id"
                            onChange={handleDropdownChange}
                            onBlur={handleChange}
                            fullWidth
                            required
                        >
                            {exhibits.map((exhibit) => (
                                <MenuItem key={exhibit.id} value={exhibit.id}>
                                    {exhibit.title}
                                </MenuItem>
                            ))}
                        </TextField>
                        <TextField
                            label="Title"
                            type="text"
                            variant="outlined"
                            name="title" 
                            onBlur={handleChange} 
                            fullWidth
                            required
                        />
                        <TextField
                            label="Author"
                            type="text"
                            variant="outlined"
                            name="author" 
                            onBlur={handleChange} 
                            fullWidth
                        />
                        <TextField
                            label="Description"
                            type="text"
                            multiline
                            rows={5}
                            variant="outlined"
                            name="description" 
                            onBlur={handleChange} 
                            fullWidth
                        />
                        <TextField
                            label="Origin"
                            type="text"
                            variant="outlined"
                            name="origin" 
                            onBlur={handleChange} 
                            fullWidth
                        />
                        <TextField
                            label="Era"
                            type="text"
                            variant="outlined"
                            name="era" 
                            onBlur={handleChange} 
                            fullWidth
                        />
                        <TextField
                            label="Acquistion Date"
                            type="date"
                            variant="outlined"
                            name="acquistion_date" 
                            onBlur={handleChange} 
                            InputLabelProps={{ shrink: true }}
                            fullWidth
                        />
                        <div className="piece-modal-content-dimension-container">
                            <TextField
                                label="Dimensions"
                                placeholder="Length"
                                name="0"
                                type="number"
                                variant="outlined"
                                InputProps={{
                                    endAdornment: <InputAdornment position="end">in.</InputAdornment>
                                }}
                                InputLabelProps={{shrink:true}}
                                onBlur={handleDimensionChange}
                            />
                            <p className="piece-modal-content-dimension-dash">:</p>
                            <TextField
                                placeholder="Width"
                                name="1"
                                type="number"
                                variant="outlined"
                                InputProps={{
                                    endAdornment: <InputAdornment position="end">in.</InputAdornment>
                                }}
                                onBlur={handleDimensionChange}
                            />
                            <p className="piece-modal-content-dimension-dash">:</p>
                            <TextField
                                placeholder="Height"
                                name="2"
                                type="number"
                                variant="outlined"
                                InputProps={{
                                    endAdornment: <InputAdornment position="end">in.</InputAdornment>
                                }}
                                onBlur={handleDimensionChange}
                            />
                        </div>
                        <div className="piece-modal-content-coordinates-container">
                            <TextField
                                label="Coordinates"
                                placeholder="X"
                                name="0"
                                type="number"
                                variant="outlined"
                                InputLabelProps={{shrink:true}}
                                InputProps={{
                                    startAdornment: <InputAdornment position="start">(</InputAdornment>
                                }}
                                onBlur={handleCoordinateChange}
                            />
                            <p className="piece-modal-content-coordinates-comma">,</p>
                            <TextField
                                placeholder="Y"
                                name="1"
                                type="number"
                                variant="outlined"
                                InputProps={{
                                    endAdornment: <InputAdornment position="end">)</InputAdornment>
                                }}
                                onBlur={handleCoordinateChange}
                            />
                        </div>
                    </div>
                </DialogContent>
                <DialogActions>
                    <div className="piece-modal-buttons">
                        <div className="piece-modal-button-cancel">
                            <Button onClick={handleOpenAlert} variant="outlined">
                                Cancel
                            </Button>
                        </div>
                        <div className="piece-modal-button-add">
                            <Button onClick={handleSave} variant="outlined">
                                Add
                            </Button>
                        </div>
                    </div>
                </DialogActions>
            </Dialog>
             <Dialog open={openDeleteAlert} onClose={handleCloseAlert}>
                <DialogTitle>Quit editing?</DialogTitle>
                <DialogActions>
                    <div className="piece-modal-delete-alert-buttons">
                        <div className="piece-modal-delete-alert-button-delete">
                            <Button onClick={handleDeleteClick} variant="outlined">
                            Delete
                            </Button>
                        </div>
                        <div className="piece-modal-delete-alert-button-cancel">
                            <Button onClick={handleCloseAlert} variant="outlined">
                                Cancel
                            </Button>
                        </div>
                    </div>
                </DialogActions>
            </Dialog>
        </div>
    )
}

export default PieceButton;