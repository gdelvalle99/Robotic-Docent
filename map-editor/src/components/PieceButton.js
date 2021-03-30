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
import Add from '@material-ui/icons/Add';

export const PieceButton = () => {
    const [open, setOpen] = useState(false);

    const handleClose = () => {
        setOpen(false);
    }

    const handleOpenClick = () => {
        setOpen(true);
    }
    
    return (
        <div>
            <Fab color="primary" aria-label="add" className="floating-piece-button" onClick={handleOpenClick}>
                <Add fontSize='large'/>
            </Fab>
            <Dialog open={open} onClose={handleClose} aria-labelledby="form-dialog-title">
                <DialogTitle id="form-dialog-title">Add a Museum Display</DialogTitle>
                <DialogContent>
                <InputLabel id="demo-simple-select-label">Exhibit Location</InputLabel>
                <Select
                labelId="demo-simple-select-label"
                id="demo-simple-select"
                style={{width: '100%'}}
                >
                    <MenuItem value={10}>Modern Art</MenuItem>
                    <MenuItem value={30}>Ancient Classical Art</MenuItem>
                </Select>
                <TextField
                    autoFocus
                    margin="dense"
                    id="name"
                    label="Title"
                    type="text"
                    fullWidth
                />
                <TextField
                    autoFocus
                    margin="dense"
                    id="name"
                    label="Author"
                    type="text"
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
                    rows={3}
                />
                <TextField
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
                />
                <InputLabel shrink>Acquisition Date</InputLabel>
                <TextField
                    autoFocus
                    margin="dense"
                    id="name"
                    type="date"
                    
                    fullWidth
                />
                <TextField
                    autoFocus
                    margin="dense"
                    id="name"
                    label="Dimensions"
                    type="text"
                    fullWidth
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
                    <Button onClick={handleClose} color="primary">
                        Add
                    </Button>
                </DialogActions>
            </Dialog>
        </div>
    )
}

export default PieceButton;