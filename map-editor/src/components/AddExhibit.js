import React, { useState, useEffect } from "react";
import { Fab } from "@material-ui/core";
import { Button } from "@material-ui/core";
import { Dialog } from "@material-ui/core";
import { TextField } from "@material-ui/core";
import { DialogActions } from "@material-ui/core";
import { DialogContent } from "@material-ui/core";
import { DialogTitle } from "@material-ui/core";
import { Select } from "@material-ui/core";
import { MenuItem } from "@material-ui/core";
import { InputLabel } from "@material-ui/core";
import { mapLink, floor_id } from "../links";
import Add from "@material-ui/icons/Add";
// import RoomIcon from '@material-ui/icons/Room';
import axios from "axios";
// import { Room } from "@material-ui/icons";
import SvgIcon from "@material-ui/core/SvgIcon";

export const AddExhibit = ({
    open,
    handleClose,
    coords,
    handleLocation,
    addToPlaces,
}) => {
    const [p, setP] = useState({
        title: "",
        coords,
        author: "",
        description: "",
        img: "",
    });

    // console.log("in add exhibit", coords, p.coords);

    const handleSave = async () => {
        // try to save, if success then follow
        // setP(prev=>{return{...prev, coords}})
        // setP({title:})
        addToPlaces(p);
        // eraseForm();
        handleClose();
    };

    return (
        <Dialog
            open={open}
            onClose={handleClose}
            aria-labelledby="form-dialog-title"
        >
            <DialogTitle id="form-dialog-title">
                Add a Museum Display
            </DialogTitle>
            <DialogContent>
                <TextField
                    autoFocus
                    margin="dense"
                    id="name"
                    label="Title"
                    type="text"
                    value={p.title}
                    onChange={(e) => setP({ ...p, title: e.target.value })}
                    fullWidth
                />
                <TextField
                    autoFocus
                    margin="dense"
                    id="name"
                    label="Author"
                    type="text"
                    value={p.author}
                    onChange={(e) => setP({ ...p, author: e.target.value })}
                    fullWidth
                />
                <TextField
                    autoFocus
                    margin="dense"
                    id="name"
                    label="Description"
                    type="text"
                    fullWidth
                    multiline
                    value={p.description}
                    onChange={(e) =>
                        setP({ ...p, description: e.target.value })
                    }
                    rows={3}
                />
                <TextField
                    autoFocus
                    margin="dense"
                    label="X Position"
                    type="number"
                    value={Math.round(coords[0] * 100) / 100 || 0}
                    required
                />
                <TextField
                    autoFocus
                    margin="dense"
                    label="Y Position"
                    type="number"
                    value={Math.round(coords[1] * 100) / 100 || 0}
                    required
                />
                <Button
                    onClick={handleLocation}
                    variant="contained"
                    color="primary"
                >
                    Choose a spot
                </Button>
                {/* <TextField
                    autoFocus
                    margin="dense"
                    id="name"
                    label="Origin"
                    type="text"
                    fullWidth
                />
                <TextField
                    autoFocus
                    margin="dense"
                    id="name"
                    label="Era"
                    type="text"
                    fullWidth
                /> */}
                <InputLabel shrink>Acquisition Date</InputLabel>
                <TextField
                    autoFocus
                    margin="dense"
                    id="name"
                    type="date"
                    fullWidth
                />
                {/* <TextField
                    autoFocus
                    margin="dense"
                    id="name"
                    label="Dimensions"
                    type="text"
                    fullWidth
                /> */}
                                <TextField
                    autoFocus
                    margin="dense"
                    label="X"
                    type="number"
                    value={Math.round(coords[1] * 100) / 100 || 0}
                    required
                />
                                <TextField
                    autoFocus
                    margin="dense"
                    label="Y"
                    type="number"
                    value={Math.round(coords[1] * 100) / 100 || 0}
                    required
                />
                                <TextField
                    autoFocus
                    margin="dense"
                    label="Z"
                    type="number"
                    value={Math.round(coords[1] * 100) / 100 || 0}
                    required
                />
                {/* <TextField
autoFocus
margin="dense"
id="name"
label="Coordinates"
type="text"
fullWidth
/> */}
            </DialogContent>
            <DialogActions>
                <Button onClick={handleClose} color="primary">
                    Cancel
                </Button>
                <Button onClick={handleSave} color="primary">
                    Add
                </Button>
            </DialogActions>
        </Dialog>
    );
};

export default AddExhibit;
